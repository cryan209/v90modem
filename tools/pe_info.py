#!/usr/bin/env python3
"""Minimal PE/COFF reader for the Windows driver work.

Written for the Motorola SM56 Boot Camp driver that serves the Apple USB Modem
(docs/apple_usb_modem_sm56.md), where the useful first question was "which of
these two drivers actually talks to USB?".  Sections plus imports answered it in
one command: USmSerial.sys imports no USBD function and contains no
IOCTL_INTERNAL_USB_SUBMIT_URB, so all USB I/O is in the 26 KB utlamot.sys.

  pe_info.py <file.sys> [--sections] [--imports] [--find-u32 0x220003 ...]

--find-u32 searches the whole file for each value as a little-endian 32-bit
word, which is how IOCTL codes and URB function constants are located.  A count
of zero is informative: it means the driver never names that constant.

No dependencies.  Handles the 32-bit PE32 drivers this work involves.
"""

import struct
import sys


class PE:
    def __init__(self, path):
        self.data = open(path, 'rb').read()
        if self.data[:2] != b'MZ':
            raise ValueError('not an MZ/PE image')
        pe = struct.unpack_from('<I', self.data, 0x3c)[0]
        if self.data[pe:pe + 4] != b'PE\0\0':
            raise ValueError('no PE signature')
        nsec, = struct.unpack_from('<H', self.data, pe + 6)
        opt_size, = struct.unpack_from('<H', self.data, pe + 20)
        opt = pe + 24
        self.machine, = struct.unpack_from('<H', self.data, pe + 4)
        self.image_base, = struct.unpack_from('<I', self.data, opt + 28)
        self.dirs = opt + 96
        self.sections = []
        off = opt + opt_size
        for _ in range(nsec):
            name = self.data[off:off + 8].rstrip(b'\0').decode('latin1')
            vsize, vaddr, rsize, raddr = struct.unpack_from('<IIII', self.data, off + 8)
            self.sections.append((name, vaddr, vsize, raddr, rsize))
            off += 40

    def offset(self, rva):
        for _name, vaddr, vsize, raddr, rsize in self.sections:
            if vaddr <= rva < vaddr + max(vsize, rsize):
                return raddr + (rva - vaddr)
        return None

    def directory(self, index):
        return struct.unpack_from('<II', self.data, self.dirs + index * 8)

    def cstring(self, rva):
        off = self.offset(rva)
        end = self.data.index(b'\0', off)
        return self.data[off:end].decode('latin1')

    def imports(self):
        rva, _size = self.directory(1)
        if not rva:
            return
        off = self.offset(rva)
        while True:
            ilt, _ts, _fc, name, iat = struct.unpack_from('<IIIII', self.data, off)
            if name == 0:
                break
            names = []
            table = self.offset(ilt or iat)
            while True:
                entry, = struct.unpack_from('<I', self.data, table)
                table += 4
                if entry == 0:
                    break
                if entry & 0x80000000:
                    names.append('ordinal %d' % (entry & 0xffff))
                else:
                    names.append(self.cstring(entry + 2))
            yield self.cstring(name), names
            off += 20


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    path = sys.argv[1]
    args = sys.argv[2:]
    want_all = not any(a.startswith('--') for a in args)
    pe = PE(path)

    if want_all or '--sections' in args:
        print('machine 0x%04x  image base 0x%x' % (pe.machine, pe.image_base))
        for name, vaddr, vsize, raddr, _rsize in pe.sections:
            print('  %-9s rva 0x%08x vsize 0x%06x raw 0x%08x' % (name, vaddr, vsize, raddr))

    if want_all or '--imports' in args:
        print('imports:')
        for dll, names in pe.imports():
            print('  %s' % dll)
            for n in names:
                print('    %s' % n)

    if '--find-u32' in args:
        for token in args[args.index('--find-u32') + 1:]:
            if token.startswith('--'):
                break
            value = int(token, 0)
            needle = struct.pack('<I', value)
            hits = []
            start = 0
            while True:
                i = pe.data.find(needle, start)
                if i < 0:
                    break
                hits.append(i)
                start = i + 1
            rvas = [pe.offset and hex(i) for i in hits[:8]]
            print('  0x%08x: %d occurrence(s)%s'
                  % (value, len(hits), ('  at file ' + ' '.join(rvas)) if hits else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
