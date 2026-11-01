"""Execute original HSF MNP5/V.42bis routines; no codec substitutions.

Optional test dependencies: unicorn, pyelftools, and a locally supplied HSF
reference object. Only memory/byte queues and invariant-check logging are host
callbacks. The modem and native tests have no dependency on this tool.
"""

import argparse
import random
import struct
import hashlib
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_X86, UC_MODE_32, UC_HOOK_CODE
from unicorn.x86_const import UC_X86_REG_ESP, UC_X86_REG_EIP, UC_X86_REG_EAX


class Oracle:
    def __init__(self, path):
        if (
            hashlib.sha256(Path(path).read_bytes()).hexdigest()
            != "bfc77118f2fcd8349b83ad2174d1fc3be660a87526d7e3c744e436cc1404e9b8"
        ):
            raise ValueError("unsupported HSF reference image (symbol ABI differs)")
        self.u = Uc(UC_ARCH_X86, UC_MODE_32)
        self.syms = {}
        self.hooks = {}
        self.output = bytearray()
        self.input = []
        u = self.u
        self.ctx = 0x3000000
        u.mem_map(self.ctx, 0x20000)
        u.mem_map(0x4000000, 0x10000)
        u.mem_map(0x5000000, 0x10000)
        self.ret = 0x5000000
        u.mem_write(self.ret, b"\x90")
        base = 0x100000
        with open(path, "rb") as f:
            e = ELFFile(f)
            sections = list(e.iter_sections())
            bases = {}
            for i, s in enumerate(sections):
                if s["sh_flags"] & 2 and s["sh_size"]:
                    size = (s["sh_size"] + 4095) & ~4095
                    u.mem_map(base, size)
                    bases[i] = base
                    if s["sh_type"] != "SHT_NOBITS":
                        u.mem_write(base, s.data())
                    base += size + 4096
            tab = e.get_section_by_name(".symtab")
            symbols = list(tab.iter_symbols())
            ext = 0x5000100

            def address(s):
                nonlocal ext
                sec = s["st_shndx"]
                if sec == "SHN_ABS":
                    return s["st_value"]
                if sec in bases:
                    return bases[sec] + s["st_value"]
                if s.name not in self.syms:
                    self.syms[s.name] = ext
                    ext += 16
                return self.syms[s.name]

            for s in symbols:
                if s.name:
                    self.syms[s.name] = address(s)
            for relsec in sections:
                if relsec["sh_type"] != "SHT_REL" or relsec["sh_info"] not in bases:
                    continue
                for r in relsec.iter_relocations():
                    at = bases[relsec["sh_info"]] + r["r_offset"]
                    a = struct.unpack("<I", u.mem_read(at, 4))[0]
                    v = address(symbols[r["r_info_sym"]])
                    t = r["r_info_type"]
                    if t == 1:
                        out = v + a
                    elif t == 2:
                        out = v + a - at
                    else:
                        raise RuntimeError("relocation " + str(t))
                    u.mem_write(at, struct.pack("<I", out & 0xFFFFFFFF))
        for name, addr in self.syms.items():
            if name.endswith("OsMemSet"):
                self.hooks[addr] = ("memset", name)
            elif name.endswith("OsMemCpy"):
                self.hooks[addr] = ("memcpy", name)
            elif addr >= 0x5000100:
                self.hooks[addr] = ("unexpected", name)
        for name, kind in [
            ("hsfengine2237_", "get"),
            ("hsfengine2238_", "put"),
            ("hsfengine2241_", "repeat"),
            ("hsfengine1340_", "check"),
        ]:
            self.hooks[self.syms[name]] = (kind, name)
        u.hook_add(UC_HOOK_CODE, self.hook)
        self.call("hsfengine1344_", self.ctx)

    def word(self, p):
        return struct.unpack("<I", self.u.mem_read(p, 4))[0]

    def setword(self, p, n):
        self.u.mem_write(p, struct.pack("<I", n))

    def hook(self, u, addr, size, user):
        if addr not in self.hooks:
            return
        kind, name = self.hooks[addr]
        sp = u.reg_read(UC_X86_REG_ESP)
        arg = lambda i: self.word(sp + 4 * i)
        result = 0
        if kind == "memset":
            u.mem_write(arg(1), bytes([arg(2) & 255]) * arg(3))
            result = arg(1)
        elif kind == "memcpy":
            u.mem_write(arg(1), bytes(u.mem_read(arg(2), arg(3))))
            result = arg(1)
        elif kind == "get":
            result = self.input.pop(0)
            self.setword(self.ctx + 0x44, 0)
        elif kind == "put":
            self.output.append(arg(2) & 255)
        elif kind == "repeat":
            self.output.extend(bytes([arg(2) & 255]) * arg(3))
        elif kind == "check":
            pass  # Original assertion routine only; no codec state changes.
        else:
            raise RuntimeError("unexpected reference callback " + name)
        u.reg_write(UC_X86_REG_EAX, result)
        u.reg_write(UC_X86_REG_ESP, sp + 4)
        u.reg_write(UC_X86_REG_EIP, self.word(sp))

    def call(self, name, *args):
        sp = 0x400F000
        self.u.mem_write(sp, struct.pack("<" + "I" * (len(args) + 1), self.ret, *args))
        self.u.reg_write(UC_X86_REG_ESP, sp)
        self.u.emu_start(self.syms[name], self.ret, count=1000000)
        if self.u.reg_read(UC_X86_REG_EIP) != self.ret:
            raise RuntimeError("instruction budget exhausted")

    def encode(self, data, flush=True):
        out = bytearray()
        for v in data:
            self.input = [v]
            self.setword(self.ctx + 0x44, 1)
            self.call("hsfengine1342_", self.ctx)
            n = self.word(self.ctx + 0x135BC)
            out.extend(self.u.mem_read(self.ctx + 0x135E4, n))
            self.setword(self.ctx + 0x135BC, 0)
        if flush:
            self.setword(self.ctx + 0x44, 0)
            self.call("hsfengine1342_", self.ctx)
            n = self.word(self.ctx + 0x135BC)
            out.extend(self.u.mem_read(self.ctx + 0x135E4, n))
            self.setword(self.ctx + 0x135BC, 0)
        return bytes(out)

    def decode(self, data):
        self.output.clear()
        for v in data:
            self.call("hsfengine1341_", self.ctx, v)
        return bytes(self.output)


def vectors(path, destination):
    rng = random.Random(0x4D4E5035)
    cases = [
        (f"run-{n}", [b"A" * n])
        for n in (0, 1, 2, 3, 4, 252, 253, 254, 255, 256, 505, 506, 507)
    ]
    cases += [
        ("alphabet", [bytes(range(256)), bytes(range(255, -1, -1))]),
        ("frequency-rescale", [b"AB" * 4096, b"CDAB" * 1024, b"AB" * 4096]),
        ("binary", [bytes(rng.getrandbits(8) for _ in range(8192)) for _ in range(2)]),
        (
            "idle-and-run-boundaries",
            [b"AAA", b"", b"AAA", b"A", b"AB", b"B" * 253, b"", b"ABCD"],
        ),
        ("all-run-lengths", [bytes([n]) * n for n in range(256)]),
    ]
    lines = [
        "# MNP5: original HSF i386 execution, SHA256 bfc77118f2fcd8349b83ad2174d1fc3be660a87526d7e3c744e436cc1404e9b8",
        "# ! starts a fresh dictionary; other lines are plaintext-hex wire-hex, each flushed. - is empty.",
    ]
    total = 0
    for name, segments in cases:
        tx, rx = Oracle(path), Oracle(path)
        lines.append("! " + name)
        for data in segments:
            wire = tx.encode(data)
            decoded = rx.decode(wire)
            if decoded != data:
                raise AssertionError("reference round trip: " + name)
            lines.append((data.hex() or "-") + " " + (wire.hex() or "-"))
            total += len(data)
        print("reference:", name, flush=True)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(lines) + "\n")
    print("Wrote", total, "plaintext bytes to", destination)


def check_v42bis(path, dump, p1, p2):
    # This legacy reference has fixed-size dictionary/string storage, unlike
    # our codec. Do not run its instructions with parameters beyond that ABI.
    if p1 not in (512, 1024, 2048, 4096) or not 6 <= p2 <= 32:
        raise ValueError("HSF reference supports P1=512/1024/2048/4096, P2=6..32")
    records = [line.split() for line in dump.read_text().splitlines()]
    wire = b"".join(bytes.fromhex(r[1]) for r in records if r[0] == "LAPM")
    expected = b"".join(bytes.fromhex(r[1]) for r in records if r[0] == "DTE")
    if not wire or not expected:
        raise ValueError("need both DTE and LAPM dump records")
    o = Oracle(path)
    # Supply writable control-function storage for the original C-ERROR path.
    o.setword(o.ctx + 0x30A8, o.ctx + 0x18000)
    o.setword(o.ctx + 0x5568, p1)
    o.setword(o.ctx + 0x556C, p2)
    o.setword(o.ctx + 0xC590, 2)
    o.call("hsfengine2225_", o.ctx)
    for i, value in enumerate(wire):
        start = len(o.output)
        o.call("hsfengine2226_", o.ctx, value)
        if o.word(o.ctx + 0x4410):
            raise ValueError(f"HSF C-ERROR at wire offset {i}")
        if o.output[start:] != expected[start : len(o.output)]:
            raise AssertionError(
                f"HSF decode mismatch at wire offset {i}, DTE offset {start}"
            )
    print(
        "EXACT:",
        len(o.output),
        "original-HSF decoded DTE-prefix bytes;",
        len(expected) - len(o.output),
        "captured DTE bytes beyond the decoded prefix",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--object",
        required=True,
        help="locally supplied original hsfengine-i386.O (not redistributed)",
    )
    parser.add_argument(
        "--vectors", type=Path, help="generate deterministic MNP5 test vectors"
    )
    parser.add_argument(
        "--v42bis-dump",
        type=Path,
        help="check DS_TX_FRAME_DUMP against original V.42bis decoder",
    )
    parser.add_argument("--p1", type=int, default=1024)
    parser.add_argument("--p2", type=int, default=32)
    args = parser.parse_args()
    if args.vectors:
        vectors(args.object, args.vectors)
    if args.v42bis_dump:
        check_v42bis(args.object, args.v42bis_dump, args.p1, args.p2)
    if not args.vectors and not args.v42bis_dump:
        parser.error("choose --vectors or --v42bis-dump")
