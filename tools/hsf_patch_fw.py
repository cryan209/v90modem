#!/usr/bin/env python3
"""Patch the HSF controller firmware to run our own 8051 code on the device.

The script interpreter's dispatch table lives in the UPLOADED image at 0x62 --
105 LJMP entries, opcode N at 0x62 + 3N -- so repointing an entry runs our code
for that opcode.  The firmware is volatile, so the worst case is a replug.

Why this is needed: the host can read the F4/F5 control register file with
opcode 0x03, but that file is only ~0x40 registers (0x40-0x5f and 0x60-0x7f read
identical, i.e. it mirrors) and is not code memory.  Nothing in the shipped
opcode set reads code memory, so the mask ROM at 0xd000+ cannot be dumped
without putting our own MOVC on the device.

The obstacle is that the "result byte" -- the accumulator that opcode 0x27 sets
and opcode 0x26 appends to the completion notification -- is managed by mask ROM
and its address is unknown.  --find-result locates it: it repoints one opcode to
a stub that writes a marker into a different internal RAM address on each call,
so the call where that address IS the result byte comes back with the marker
instead of what 0x27 put there.
"""
import argparse, sys

DISPATCH = 0x62
MARKER = 0xC3


def crc16_ccitt(b, init=0xFFFF):
    c = init
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def fix_crc(img):
    """The image carries a CRC-16-CCITT (poly 0x1021, init 0xFFFF) over
    everything but its last two bytes, and the bootloader checks it: on the
    stock image that is 0x05b9, which is exactly the trailing two bytes.  Both
    earlier patch attempts were refused with EIO for want of this -- first
    blamed on the size change, which was wrong, since a size-neutral patch was
    refused too."""
    img = bytearray(img)
    c = crc16_ccitt(bytes(img[:-2]))
    img[-2] = (c >> 8) & 0xFF
    img[-1] = c & 0xFF
    return bytes(img)


def entry_off(op):
    return DISPATCH + 3 * op


def find_result_stub(start):
    """MOV A,7Eh / JNZ skip / MOV 7Eh,#start / skip: MOV R0,7Eh /
       MOV @R0,#C3h / INC 7Eh / RET

    The sweep cursor lives at 0x7E and self-initialises, because the firmware
    zeroes internal RAM at startup and we have no clean hook there."""
    # R0 MUST be preserved: the opcode's original handler begins "INC @R0", so
    # R0 is the interpreter's own pointer.  The first version of this stub
    # clobbered it and the interpreter ran away -- every completion came back as
    # five 0xAA bytes instead of the expected three, which is what gave it away.
    return bytes([0xE5, 0x7E,           # MOV A,7Eh
                  0x70, 0x03,           # JNZ skip
                  0x75, 0x7E, start,    # MOV 7Eh,#start
                  0xC0, 0x00,           # skip: PUSH 00h  (R0)
                  0xA8, 0x7E,           # MOV R0,7Eh
                  0x76, MARKER,         # MOV @R0,#C3h
                  0x05, 0x7E,           # INC 7Eh
                  0xD0, 0x00,           # POP 00h
                  0x22])                # RET


def patch(img, op, code):
    """Overwrite the opcode's OWN handler, in place.

    Appending the stub and repointing the dispatch entry is the obvious way and
    the bootloader refuses it: growing the image by 14 bytes makes
    CD2_UPLOAD_FIRMWARE fail with EIO, and the image has no filler runs to
    borrow either (longest run of 00 or ff is under 8 bytes).  So the patch has
    to be size-neutral, and the handler being replaced is exactly the space to
    put it -- the dispatch entry already points there, so nothing else moves.
    """
    img = bytearray(img)
    e = entry_off(op)
    if img[e] != 0x02:
        sys.exit(f"dispatch entry for op 0x{op:02x} is not an LJMP")
    addr = (img[e + 1] << 8) | img[e + 2]
    if addr >= len(img):
        sys.exit(f"op 0x{op:02x} is handled in mask ROM (0x{addr:04x}); "
                 f"pick one handled inside the image")
    # room is up to the next handler that starts after this one
    nxt = min([(img[entry_off(o) + 1] << 8) | img[entry_off(o) + 2]
               for o in range(0x69)
               if ((img[entry_off(o) + 1] << 8) | img[entry_off(o) + 2]) > addr
               and ((img[entry_off(o) + 1] << 8) | img[entry_off(o) + 2]) < len(img)]
              or [len(img)])
    room = nxt - addr
    if len(code) > room:
        sys.exit(f"stub is {len(code)} bytes, only {room} free at 0x{addr:04x}")
    img[addr:addr + len(code)] = code
    return fix_crc(img), addr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("-o", "--out", default="hsf_rom_patched.bin")
    ap.add_argument("--op", type=lambda x: int(x, 0), default=0x23,
                    help="opcode to repoint (default 0x23, a no-operand op "
                         "whose handler is already inside the image)")
    ap.add_argument("--find-result", type=lambda x: int(x, 0), metavar="START",
                    help="emit the result-byte finder, sweeping from START")
    a = ap.parse_args()

    img = open(a.image, "rb").read()
    if a.find_result is None:
        sys.exit("nothing to do: pass --find-result START")
    code = find_result_stub(a.find_result)
    out, addr = patch(img, a.op, code)
    open(a.out, "wb").write(out)
    print(f"op 0x{a.op:02x} -> 0x{addr:04x} ({len(code)} bytes), "
          f"sweep starts at internal RAM 0x{a.find_result:02x}")
    print(f"wrote {a.out} ({len(out)} bytes, was {len(img)})")


if __name__ == "__main__":
    main()
