#!/usr/bin/env python3
"""Check the C PER engine against asn1tools, an independent X.691 implementation.

  per_oracle.py <h245.asn> <per_tool> [iterations] [seed]

Two directions, byte for byte and value for value:
  * the oracle encodes a value -> per_tool decodes it -> the JSON must equal the value
  * per_tool encodes that JSON -> the hex must equal the oracle's encoding
Values are random walks over the DSVD subset (tools/h245/prune.py), so what is
fuzzed is what is generated.  Needs `pip install asn1tools`.
"""
import json
import os
import random
import subprocess
import sys

import asn1tools

sys.path.insert(0, os.path.dirname(__file__))
import prune  # noqa: E402

BUILTIN = {'BOOLEAN', 'NULL', 'INTEGER', 'ENUMERATED', 'OCTET STRING',
           'OBJECT IDENTIFIER', 'SEQUENCE', 'SEQUENCE OF', 'SET', 'SET OF', 'CHOICE'}


def resolves_to_null(types, node):
    t = node['type']
    while t not in BUILTIN:
        node = types[t]
        t = node['type']
    return t == 'NULL'


class Walker:
    def __init__(self, types, rng):
        self.types = types
        self.rng = rng

    def value(self, node, path):
        t = node['type']
        if t not in BUILTIN:
            node = self.types[t]
            path = t
            t = node['type']
        r = self.rng
        if t == 'BOOLEAN':
            return r.random() < 0.5
        if t == 'NULL':
            return None
        if t == 'INTEGER':
            lb, ub = node['restricted-to'][0]
            pick = r.choice(['lb', 'ub', 'lb1', 'ub1', 'rand', 'rand', 'small'])
            if pick == 'lb': return lb
            if pick == 'ub': return ub
            if pick == 'lb1': return min(lb + 1, ub)
            if pick == 'ub1': return max(ub - 1, lb)
            if pick == 'small': return min(lb + r.randint(0, 70), ub)
            return r.randint(lb, ub)
        if t == 'OBJECT IDENTIFIER':
            return '.'.join(str(x) for x in [0, 0, 8, 245, 0] + [r.randint(0, 300)])
        if t == 'OCTET STRING':
            lb, ub = node['size'][0]
            n = r.randint(lb, min(ub, lb + 6))
            return bytes(r.randint(0, 255) for _ in range(n))
        if t in ('SEQUENCE OF', 'SET OF'):
            lb, ub = node['size'][0] if node.get('size') else (0, 8)
            hi = min(ub, lb + 3) if r.random() < 0.9 else min(ub, lb + 40)
            n = r.randint(lb, hi)
            return [self.value(node['element'], path + '.elem') for _ in range(n)]
        if t == 'SEQUENCE':
            members = node['members']
            ext_at = members.index(None) if None in members else len(members)
            out = {}
            for i, m in enumerate([x for x in members if x is not None]):
                mp = path + '.' + m['name']
                if mp in prune.MEMBER_DROP:
                    continue
                is_ext = i >= ext_at
                if is_ext and resolves_to_null(self.types, m):
                    continue        # asn1tools writes an empty open type for NULL; X.691 10.1.3 says one 00 octet
                if m.get('optional') or is_ext:
                    if r.random() > (0.3 if is_ext else 0.55):
                        continue
                out[m['name']] = self.value(m, mp)
            return out
        if t == 'CHOICE':
            alts = [m for m in node['members'] if m is not None]
            keep = prune.CHOICE_KEEP.get(path)
            if keep is not None:
                alts = [m for m in alts if m['name'] in keep]
            ext_at = [x for x in node['members']].index(None) if None in node['members'] else 10**9
            real = [x for x in node['members'] if x is not None]
            alts = [m for m in alts if not (real.index(m) >= ext_at and resolves_to_null(self.types, m))]
            m = r.choice(alts)
            return (m['name'], self.value(m, path + '.' + m['name']))
        raise SystemExit('fuzzer: unsupported %s at %s' % (t, path))


def to_json(v):
    if isinstance(v, tuple):
        return [v[0], to_json(v[1])]
    if isinstance(v, dict):
        return {k: to_json(x) for k, x in v.items()}
    if isinstance(v, list):
        return [to_json(x) for x in v]
    if isinstance(v, (bytes, bytearray)):
        return bytes(v).hex()
    return v


def run_tool(tool, mode, typename, lines):
    p = subprocess.run([tool, mode, typename], input='\n'.join(lines) + '\n',
                       capture_output=True, text=True)
    return p.stdout.strip().split('\n')


def main():
    asn, tool = sys.argv[1], sys.argv[2]
    iters = int(sys.argv[3]) if len(sys.argv) > 3 else 2000
    seed = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    rng = random.Random(seed)
    spec = asn1tools.compile_files(asn, 'per')
    types = asn1tools.parse_files(asn)['MULTIMEDIA-SYSTEM-CONTROL']['types']
    w = Walker(types, rng)
    bad = 0
    total = 0
    targets = ['MultimediaSystemControlMessage', 'V76LogicalChannelParameters',
               'DataApplicationCapability', 'AudioCapability', 'Capability',
               'TerminalCapabilitySet', 'OpenLogicalChannel', 'V76Capability',
               'RequestMode', 'EndSessionCommand', 'CloseLogicalChannel']
    for tn in targets:
        n = iters if tn == 'MultimediaSystemControlMessage' else max(iters // 8, 100)
        vals, encs = [], []
        for _ in range(n):
            v = w.value({'type': tn}, tn)
            try:
                encs.append(spec.encode(tn, v).hex())
            except Exception as e:      # an oracle refusal: report, do not hide
                print('ORACLE REFUSED %s: %s\n   %r' % (tn, str(e)[:200], v))
                bad += 1
                continue
            vals.append(v)
        decoded = run_tool(tool, 'dump', tn, encs)
        encoded = run_tool(tool, 'enc', tn, [json.dumps(to_json(v)) for v in vals])
        for v, e, d, c in zip(vals, encs, decoded, encoded):
            total += 1
            want = to_json(v)
            ok_dec = not d.startswith('ERR') and json.loads(d) == json.loads(json.dumps(want))
            ok_enc = c == e
            if not (ok_dec and ok_enc):
                bad += 1
                if bad <= 8:
                    print('MISMATCH %s dec=%s enc=%s\n  value  %s\n  oracle %s\n  C enc  %s\n  C dec  %s' % (
                        tn, ok_dec, ok_enc, json.dumps(want)[:300], e, c, d[:300]))
        print('%-34s %5d values  %s' % (tn, len(vals), 'ok' if bad == 0 else '(mismatches so far: %d)' % bad))
    print('%d values compared, %d failures' % (total, bad))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
