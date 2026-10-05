#!/usr/bin/env python3
"""Generate the C schema tables for the DSVD subset of H.245.

  per_gen.py <h245.asn> <out.c> <out.h>

Walks the parsed ASN.1 of MULTIMEDIA-SYSTEM-CONTROL from the root message
type, following references, and emits constant tables that per.c interprets
(aligned PER, X.691).  See prune.py for what is kept and why pruning is safe.
Any construct this does not understand stops it with an error: a table that
silently differs from the ASN.1 would encode wrong bits.
"""
import os
import re
import sys

import asn1tools

sys.path.insert(0, os.path.dirname(__file__))
import prune  # noqa: E402

BUILTIN = {'BOOLEAN', 'NULL', 'INTEGER', 'ENUMERATED', 'OCTET STRING',
           'OBJECT IDENTIFIER', 'SEQUENCE', 'SEQUENCE OF', 'SET', 'SET OF',
           'CHOICE'}


def die(msg):
    sys.exit('per_gen: ' + msg)


class Gen:
    def __init__(self, types):
        self.types = types
        self.done = {}          # path -> C symbol
        self.defs = []          # (symbol, member_lines, type_line)
        self.named = {}         # ASN.1 type name -> C symbol
        self.kept = 0

    @staticmethod
    def sym(path):
        return 'T_' + re.sub(r'[^A-Za-z0-9]', '_', path)

    def bound(self, v, what):
        if isinstance(v, int):
            return v, True
        if v in ('MIN', 'MAX'):
            return 0, False
        die('unsupported bound %r in %s' % (v, what))

    def range_of(self, node, key, what):
        """(has_lb, lb, has_ub, ub) from restricted-to / size."""
        if key not in node or not node[key]:
            return 0, 0, 0, 0
        r = node[key]
        if len(r) != 1 or not isinstance(r[0], tuple):
            die('%s: constraint %r is not a single range' % (what, r))
        lb, ub = r[0]
        l, hl = self.bound(lb, what)
        u, hu = self.bound(ub, what)
        if lb == 'MIN' and key == 'restricted-to':
            die('%s: negative-unbounded INTEGER not needed' % what)
        return int(hl), l, int(hu), u

    def ref(self, node, path):
        """C symbol for the type a member/definition node describes."""
        t = node['type']
        if t not in BUILTIN:                            # a reference
            if t not in self.types:
                die('unknown type %s at %s' % (t, path))
            sym = self.gen(self.types[t], t)
            self.named[t] = sym
            return sym
        return self.gen(node, path)

    def gen(self, node, path):
        if path in self.done:
            return self.done[path]
        if node['type'] in ('BOOLEAN', 'NULL') and '.' in path:
            return 'T_' + node['type']                  # shared leaf (defined in per.c)
        sym = self.sym(path)
        self.done[path] = sym
        kind = node['type']
        k = None
        flags = 'extensible'
        hl = lb = hu = ub = 0
        nroot = ntot = 0
        mem = []
        elem = '0'
        if kind == 'BOOLEAN':
            k = 'A_BOOL'
        elif kind == 'NULL':
            k = 'A_NULL'
        elif kind == 'OBJECT IDENTIFIER':
            k = 'A_OID'
        elif kind == 'INTEGER':
            k = 'A_INT'
            hl, lb, hu, ub = self.range_of(node, 'restricted-to', path)
        elif kind == 'OCTET STRING':
            k = 'A_OCTETS'
            hl, lb, hu, ub = self.range_of(node, 'size', path)
        elif kind in ('SEQUENCE OF', 'SET OF'):
            k = 'A_SEQOF'
            hl, lb, hu, ub = self.range_of(node, 'size', path)
            elem = self.ref(node['element'], path + '.elem')
        elif kind in ('SEQUENCE', 'CHOICE'):
            k = 'A_SEQ' if kind == 'SEQUENCE' else 'A_CHOICE'
            members = node['members']
            ext_at = None
            for i, m in enumerate(members):
                if m is None:
                    if ext_at is not None:
                        die('%s: a second extension marker (root after additions)' % path)
                    ext_at = i
            real = [m for m in members if m is not None]
            nroot = ext_at if ext_at is not None else len(real)
            ntot = len(real)
            keep = prune.CHOICE_KEEP.get(path) if kind == 'CHOICE' else None
            for i, m in enumerate(real):
                mpath = path + '.' + m['name']
                if 'default' in m:
                    die('%s: DEFAULT components are not needed and not implemented' % mpath)
                opt = 1 if m.get('optional') else 0
                drop = False
                if kind == 'CHOICE' and keep is not None and m['name'] not in keep:
                    drop = True
                if kind == 'SEQUENCE' and mpath in prune.MEMBER_DROP:
                    drop = True
                if kind == 'SEQUENCE' and i >= nroot:
                    # Every extension addition has a presence bit in the
                    # extension bitmap whether or not it says OPTIONAL: a
                    # sender of an earlier version simply does not send it.
                    opt = 1
                msym = '0' if drop else '&' + self.ref(m, mpath)
                if not drop:
                    self.kept += 1
                mem.append('    { "%s", %s, %d },' % (m['name'], msym, opt))
        elif kind == 'ENUMERATED':
            die('%s: ENUMERATED is not in the DSVD subset' % path)
        else:
            die('%s: unsupported type %s' % (path, kind))
        ext = 1 if (kind in ('SEQUENCE', 'CHOICE') and None in node['members']) else 0
        self.defs.append((sym, mem, '%s, %d, %d, %d, %d, %dLL, %dLL, %d, %d, %s' % (
            k, ext, hl, hu, 0, lb, ub, nroot, ntot, elem), path))
        return sym

    def emit(self, root):
        sym = self.ref({'type': root}, root)
        self.named[root] = sym
        decl = []
        out = ['/* GENERATED by tools/h245/per_gen.py from the MULTIMEDIA-SYSTEM-CONTROL',
               ' * module of ITU-T H.245 (03/2022).  Do not edit; see tools/h245/prune.py for',
               ' * the subset.  Dropped alternatives are counted but have no schema ("0"). */',
               '#include "per.h"', '']
        for s, mem, tl, path in self.defs:
            decl.append('extern const a_type_t %s;' % s)
        for s, mem, tl, path in self.defs:
            if mem:
                out.append('static const a_member_t M_%s[] = {' % s)
                out.extend(mem)
                out.append('};')
            k, ext, hl, hu, _z, lb, ub, nroot, ntot, elem = [x.strip() for x in tl.split(',', 9)]
            out.append('const a_type_t %s = { %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, "%s" };' % (
                s, k, ext, hl, hu, lb, ub, nroot, ntot, ('M_' + s) if mem else '0',
                ('&' + elem) if elem != '0' else '0', path.split('.')[-1] if '.' not in path else path))
            out[-1] = out[-1].replace(', "%s" };' % (path.split('.')[-1] if '.' not in path else path),
                                      ', "%s" };' % path)
        out.append('')
        out.append('const a_named_t h245_types[] = {')
        for n in sorted(self.named):
            out.append('    { "%s", &%s },' % (n, self.named[n]))
        out.append('    { 0, 0 }')
        out.append('};')
        return '\n'.join(out) + '\n', '\n'.join(decl) + '\n'


def main():
    asn, out_c, out_h = sys.argv[1:4]
    types = asn1tools.parse_files(asn)['MULTIMEDIA-SYSTEM-CONTROL']['types']
    g = Gen(types)
    c, decls = g.emit(prune.ROOT)
    header = ('/* GENERATED by tools/h245/per_gen.py.  Do not edit. */\n'
              '#ifndef H245_SCHEMA_H\n#define H245_SCHEMA_H\n#include "per.h"\n'
              + decls + '#endif\n')
    open(out_c, 'w').write(c)
    open(out_h, 'w').write(header)
    sys.stderr.write('per_gen: %d types, %d kept members\n' % (len(g.defs), g.kept))


if __name__ == '__main__':
    main()
