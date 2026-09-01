#!/usr/bin/env python3
"""Extract the CD2_CONTROL_SCRIPT templates from hsfusbcd2-i386.O.

The driver does not build script bodies byte by byte -- it keeps a table of
pre-baked templates and patches a few runtime values into each before sending.
That is why no guessed body was ever accepted: the real ones are tens of bytes
of static data.

Found by disassembling the one async CD2_CONTROL_SCRIPT call site (.text 0x2f18,
`pushl $0x2` = the request code) and walking back through its arguments:

    2cfc  edx = idx*24                        ; 24-byte descriptors
    2d02  ax  = word [hsfusbcd250_ + idx*24 + 4]   ; length
    2d18  edi = dword[hsfusbcd250_ + idx*24 + 0]   ; -> template
    2d1f  call builder(template, buf, &len, len)
    2e39  ecx = dword[hsfusbcd250_ + idx*24 + 0]   ; patch destination
    2e4a  mov [ecx + offset], al              ; runtime byte from ctx+0x6e0
    2e4f  al  = byte [hsfusbcd250_ + idx*24 + 6]   ; patch count

  python3 tools/hsf_scripts.py ~/hsfmodem-src/.../imported/hsfusbcd2-i386.O
"""
import argparse
import struct
import sys

TABLE_SYM = "hsfusbcd250_"
ENTRY = 24

# SMART_RELAYS from hsf.cty; the firmware ORs 0x80 in itself (ROM_IMAGE 0A88),
# so the low byte alone is as likely to appear as the full word.
RELAY_LOW = {0xA4: "PULSE_MAKE/SETUP/CLEAR", 0xA5: "ONHOOK_PHONEOFFLINE_NOCALLID",
             0xA6: "OFFHOOK_PHONEOFFLINE  <<< SEIZE", 0xAD: "ONHOOK_PHONEOFFLINE_CALLID  <<< LISTEN",
             0xB5: "GPIO_DEFAULT", 0xB6: "OFFHOOK_PHONETOLINE",
             0xBD: "ONHOOK_PHONETOLINE_CALLID"}


class ELF:
    def __init__(self, path):
        self.d = open(path, "rb").read()
        if self.d[:4] != b"\x7fELF":
            sys.exit("not an ELF file")
        (self.shoff,) = struct.unpack_from("<I", self.d, 0x20)
        self.shentsize, self.shnum, self.shstrndx = struct.unpack_from("<HHH", self.d, 0x2E)
        self.sec = []
        for i in range(self.shnum):
            o = self.shoff + i * self.shentsize
            name, typ, flags, addr, off, size, link, info, align, entsz = \
                struct.unpack_from("<10I", self.d, o)
            self.sec.append(dict(name=name, type=typ, addr=addr, off=off,
                                 size=size, link=link, info=info, entsize=entsz))
        strtab = self.sec[self.shstrndx]
        for s in self.sec:
            s["nm"] = self.cstr(strtab["off"] + s["name"])
        self.symtab = next(s for s in self.sec if s["nm"] == ".symtab")
        self.strtab = self.sec[self.symtab["link"]]
        self.syms = []
        for o in range(self.symtab["off"], self.symtab["off"] + self.symtab["size"], 16):
            nm, val, size, info, other, shndx = struct.unpack_from("<IIIBBH", self.d, o)
            self.syms.append(dict(nm=self.cstr(self.strtab["off"] + nm), val=val,
                                  size=size, shndx=shndx))

    def cstr(self, o):
        e = self.d.index(b"\0", o)
        return self.d[o:e].decode("latin1")

    def sym(self, name):
        for s in self.syms:
            if s["nm"] == name:
                return s
        return None

    def sec_by_name(self, n):
        for s in self.sec:
            if s["nm"] == n:
                return s
        return None

    def relocs(self, secname):
        """{offset_in_section: symbol} for .rel<secname>"""
        r = self.sec_by_name(".rel" + secname)
        out = {}
        if not r:
            return out
        for o in range(r["off"], r["off"] + r["size"], 8):
            roff, rinfo = struct.unpack_from("<II", self.d, o)
            out[roff] = self.syms[rinfo >> 8]
        return out

    def read(self, sec, off, n):
        return self.d[sec["off"] + off: sec["off"] + off + n]


# ---------------------------------------------------------------------------
# hsfusbcd290_ -- the template assembler.
#
# The table above holds an INTERMEDIATE language, not the wire body: the driver
# runs each template through hsfusbcd290_(.text 0x6f20) before handing it to
# OsUsbMakeControlRequest.  Two things happen there that no amount of staring at
# the templates would reveal, and both are why a template sent verbatim is
# rejected:
#
#   - opcodes 0x54/0x58/0x5c/0x60/0x64/0x68 are REMAPPED to (op - 0x51) with
#     their operand biased by +0x18 (.text 0x7031);
#   - opcodes 0x19 and 0x1b-0x22 carry a LABEL number, not an address.  The
#     assembler keeps a 64-entry label table, writes 0xff for a forward
#     reference, threads the unresolved sites through the destination buffer
#     itself as a linked list, and opcode 0x4f -- which emits NOTHING -- defines
#     a label and back-patches the whole chain (.text 0x70e3 / 0x718b).
#
# So the wire body is shorter than its template (every 0x4f disappears) and its
# branch operands are absolute offsets into the body.  Everything else is copied
# verbatim at a per-opcode fixed width, which is what the jump table at
# .rodata 0x480 encodes.
# ---------------------------------------------------------------------------

# op -> bytes copied verbatim (handler address in the original jump table).
_FIXED_WIDTH = {0x6fa4: 1, 0x701c: 3, 0x701e: 2, 0x7025: 2,
                0x70cb: 6, 0x70d7: 3, 0x71f1: 4, 0x71fd: 5}
_JUMP_TABLE_RODATA_OFF = 0x480
_JUMP_TABLE_N = 0x69


def read_jump_table(e):
    ro = e.sec_by_name(".rodata")
    return [struct.unpack_from("<I", e.d, ro["off"] + _JUMP_TABLE_RODATA_OFF + 4 * i)[0]
            for i in range(_JUMP_TABLE_N)]


class ScriptError(Exception):
    pass


def build(src, jt):
    """Assemble one template into the CD2_CONTROL_SCRIPT wire body.

    Returns (wire, srcmap), where srcmap[i] is the offset in the wire body that
    template byte i ended up at, or -1 if the assembler consumed it (a label
    definition) or rewrote it (the 0x51.. remap keeps its operand, biased).
    The driver patches the TEMPLATE and builds afterwards, so a runtime patch
    offset has to be translated through srcmap to be applied to a wire body.
    """
    src = bytes(src)
    srcmap = [-1] * len(src)
    dst = bytearray(0x100)
    label_pos = [0xFF] * 0x40      # -0x58(%ebp): chain head / defined offset
    label_fwd = [0] * 0x40         # -0x98(%ebp): non-zero while unresolved
    sp = dp = 0
    n = len(src)

    while sp < n:
        op = src[sp]
        arg = src[sp + 1] if sp + 1 < n else 0
        if op >= _JUMP_TABLE_N:
            raise ScriptError("opcode 0x%02x out of range" % op)
        h = jt[op]

        if h in _FIXED_WIDTH:
            k = _FIXED_WIDTH[h]
            dst[dp:dp + k] = src[sp:sp + k]
            for j in range(k):
                srcmap[sp + j] = dp + j
            sp += k
            dp += k
        elif h == 0x7079:                      # 0x1a: payload length at +2
            k = src[sp + 2] + 3
            dst[dp:dp + k] = src[sp:sp + k]
            for j in range(k):
                srcmap[sp + j] = dp + j
            sp += k
            dp += k
        elif h in (0x7031, 0x7033):            # opcode/operand remap
            dst[dp] = op - 0x51
            dst[dp + 1] = (arg + 0x18) & 0xFF
            srcmap[sp] = dp
            srcmap[sp + 1] = dp + 1            # NB: biased by +0x18 on the wire
            dp += 2
            sp += 2
            if h == 0x7031:                    # ...plus one verbatim byte
                dst[dp] = src[sp]
                srcmap[sp] = dp
                dp += 1
                sp += 1
        elif h == 0x7092:                      # 0x18: script header
            if dp != 0:
                raise ScriptError("opcode 0x18 not at offset 0")
            dst[0] = 0x18
            dst[1] = (arg + 4) & 0xFF
            dst[2] = arg
            dst[arg + 3] = 0x36
            dp = arg + 4
            sp += 2
        elif h == 0x718b:                      # 0x4f: define label, emit nothing
            if arg > 0x3F:
                raise ScriptError("label %d out of range" % arg)
            if label_fwd[arg]:
                link = label_pos[arg]
                while True:
                    nxt = dst[link]
                    dst[link] = dp
                    if nxt == 0xFF:
                        break
                    link = nxt
                label_fwd[arg] = 0
            label_pos[arg] = dp
            sp += 2
        elif h == 0x70e3:                      # 0x19, 0x1b-0x22: label reference
            if arg > 0x3F:
                raise ScriptError("label %d out of range" % arg)
            dst[dp] = op
            srcmap[sp] = dp
            dp += 1
            cur = label_pos[arg]
            dst[dp] = cur
            if label_fwd[arg] or cur == 0xFF:
                if not label_fwd[arg]:
                    label_fwd[arg] = arg
                label_pos[arg] = dp        # thread this site onto the chain
            dp += 1
            if op == 0x1B:
                sp += 2
            else:
                dst[dp] = src[sp + 2]
                srcmap[sp + 2] = dp
                dp += 1
                if op == 0x22:
                    dst[dp] = 0
                    dp += 1
                    sp += 3
                elif op == 0x19:               # trailing counted payload
                    cnt = src[sp + 2]
                    dst[dp:dp + cnt] = src[sp + 3:sp + 3 + cnt]
                    for j in range(cnt):
                        srcmap[sp + 3 + j] = dp + j
                    dp += cnt
                    sp += 3 + cnt
                else:
                    sp += 3
        else:
            raise ScriptError("opcode 0x%02x: handler %04x not implemented" % (op, h))

        if dp > 0x100:
            raise ScriptError("body overflows 256 bytes")

    return bytes(dst[:dp]), srcmap


# Script ids as the driver enqueues them (hsfusbcd2176_(ctx, id, wIndex, patches)),
# recovered from all 19 call sites.  The id indexes the template table directly.
# wIndex is 1 everywhere except two sites that pass 3 for script 8; hsfusbcd2210_
# additionally sends wValue 0xFF02 rather than 0xFF01 for script 8.
CALL_SITES = {
    1:  [("hsfusbcd2241_", 1), ("hsfusbcd2241_+0x80", 1)],
    2:  [("hsfusbcd2200_", 1)],
    3:  [("hsfusbcd2180_", 1), ("hsfusbcd2185_+0xda", 1)],
    4:  [("hsfusbcd2166_", 1), ("hsfusbcd2185_+0x97", 1), ("hsfusbcd2201_+0x48", 1)],
    5:  [("hsfusbcd2165_+0xfd", 1)],
    6:  [("hsfusbcd2195_", 1), ("hsfusbcd2201_+0x32", 1)],
    7:  [("hsfusbcd2220_", 1)],
    8:  [("hsfusbcd2187_", 3), ("hsfusbcd2247_", 1), ("hsfusbcd2250_", 3)],
    9:  [("hsfusbcd2165_+0xcd", 1), ("hsfusbcd2165_+0x126", 1)],
    10: [("hsfusbcd2202_", 1)],
    12: [("hsfusbcd2261_", 1)],
}


def script_wvalue(script_id):
    """hsfusbcd2210_ .text 0x2ea7: script 8 goes out as 0xFF02, everything else 0xFF01."""
    return 0xFF02 if script_id == 8 else 0xFF01


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("--header", metavar="PATH",
                    help="write the assembled wire bodies as a C header")
    args = ap.parse_args()

    e = ELF(args.obj)
    t = e.sym(TABLE_SYM)
    if not t:
        sys.exit(f"{TABLE_SYM} not found")
    data = e.sec[t["shndx"]]
    n = t["size"] // ENTRY
    jt = read_jump_table(e)
    print(f"{TABLE_SYM} at {data['nm']}+0x{t['val']:x}, {t['size']} bytes = {n} entries\n")

    drel = e.relocs(data["nm"])
    built = {}

    for i in range(n):
        base = t["val"] + i * ENTRY
        ent = e.read(data, base, ENTRY)
        addend, length, npatch = struct.unpack_from("<IHH", ent, 0)
        offs = struct.unpack_from("<8H", ent, 8)

        sym = drel.get(base)
        if sym is None:
            print(f"[{i:2d}] len={length:<4d} patches={npatch}  (no relocation -- empty slot)")
            continue

        tsec = e.sec[sym["shndx"]]
        body = e.read(tsec, sym["val"] + addend, length)

        callers = ", ".join(f"{f} wIndex={x}" for f, x in CALL_SITES.get(i, []))
        print(f"[{i:2d}] template {sym['nm'] or tsec['nm']}+0x{sym['val']+addend:x} "
              f"len={length} patches={npatch} offsets={[o for o in offs[:npatch]]}")
        if callers:
            print(f"       enqueued by {callers}, wValue=0x{script_wvalue(i):04X}")
        for o in range(0, len(body), 16):
            chunk = body[o:o+16]
            asc = "".join(chr(c) if 32 <= c < 127 else "." for c in chunk)
            print(f"       {o:04x}  {chunk.hex():<32s}  {asc}")
        for off, b in enumerate(body):
            if b in RELAY_LOW:
                print(f"       ^ byte {off} = {b:02X}  {RELAY_LOW[b]}")

        try:
            wire, srcmap = build(body, jt)
        except ScriptError as ex:
            print(f"       BUILD FAILED: {ex}\n")
            continue
        built[i] = (wire, srcmap)
        print(f"       wire len={len(wire)}")
        for o in range(0, len(wire), 16):
            chunk = wire[o:o+16]
            print(f"       {o:04x}  {chunk.hex()}")
        print()

    if args.header:
        with open(args.header, "w") as f:
            f.write("/* Generated by tools/hsf_scripts.py -- do not edit.\n"
                    " * CD2_CONTROL_SCRIPT wire bodies, assembled from the templates in\n"
                    " * hsfusbcd2-i386.O by the same procedure the vendor driver uses.\n"
                    " * Patch bytes are NOT applied here; see hsf_script_patch(). */\n\n")
            f.write("#ifndef HSF_SCRIPTS_H\n#define HSF_SCRIPTS_H\n\n#include <stdint.h>\n\n")
            f.write("struct hsf_script {\n\tconst uint8_t *body;\n\tuint16_t len;\n"
                    "\tuint16_t wvalue;\n\tuint16_t npatch;\n\tuint16_t patch_off[8];\n};\n\n")
            for i, (wire, _) in sorted(built.items()):
                f.write(f"static const uint8_t hsf_script_{i}[] = {{\n")
                for o in range(0, len(wire), 12):
                    f.write("\t" + " ".join(f"0x{b:02x}," for b in wire[o:o+12]) + "\n")
                f.write("};\n")
            f.write("\nstatic const struct hsf_script hsf_scripts[] = {\n")
            for i in range(n):
                if i not in built:
                    f.write("\t{ 0, 0, 0, 0, { 0 } },\n")
                    continue
                ent = e.read(data, t["val"] + i * ENTRY, ENTRY)
                _, _, npatch = struct.unpack_from("<IHH", ent, 0)
                offs = struct.unpack_from("<8H", ent, 8)
                srcmap = built[i][1]
                wire_offs = []
                for k, o in enumerate(offs):
                    if k >= npatch:
                        wire_offs.append(0)
                    elif o < len(srcmap) and srcmap[o] >= 0:
                        wire_offs.append(srcmap[o])
                    else:
                        print(f"[{i:2d}] WARNING: patch offset {o} does not survive assembly")
                        wire_offs.append(0xFFFF)
                f.write(f"\t{{ hsf_script_{i}, sizeof hsf_script_{i}, "
                        f"0x{script_wvalue(i):04x}, {npatch}, "
                        "{ " + ", ".join(str(o) for o in wire_offs) + " } },\n")
            f.write("};\n\n#endif /* HSF_SCRIPTS_H */\n")
        print(f"wrote {args.header}")


if __name__ == "__main__":
    main()
