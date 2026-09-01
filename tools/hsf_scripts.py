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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    args = ap.parse_args()

    e = ELF(args.obj)
    t = e.sym(TABLE_SYM)
    if not t:
        sys.exit(f"{TABLE_SYM} not found")
    data = e.sec[t["shndx"]]
    n = t["size"] // ENTRY
    print(f"{TABLE_SYM} at {data['nm']}+0x{t['val']:x}, {t['size']} bytes = {n} entries\n")

    drel = e.relocs(data["nm"])

    for i in range(n):
        base = t["val"] + i * ENTRY
        ent = e.read(data, base, ENTRY)
        addend, length, npatch = struct.unpack_from("<IHB", ent, 0)
        offs = struct.unpack_from("<8H", ent, 8)

        sym = drel.get(base)
        if sym is None:
            print(f"[{i:2d}] len={length:<4d} patches={npatch}  (no relocation -- empty slot)")
            continue

        tsec = e.sec[sym["shndx"]]
        body = e.read(tsec, sym["val"] + addend, length)

        print(f"[{i:2d}] template {sym['nm'] or tsec['nm']}+0x{sym['val']+addend:x} "
              f"len={length} patches={npatch} offsets={[o for o in offs[:npatch]]}")
        for o in range(0, len(body), 16):
            chunk = body[o:o+16]
            asc = "".join(chr(c) if 32 <= c < 127 else "." for c in chunk)
            print(f"       {o:04x}  {chunk.hex():<32s}  {asc}")
        for off, b in enumerate(body):
            if b in RELAY_LOW:
                print(f"       ^ byte {off} = {b:02X}  {RELAY_LOW[b]}")
        print()


if __name__ == "__main__":
    main()
