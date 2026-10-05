#!/usr/bin/env python3
"""Extract the MULTIMEDIA-SYSTEM-CONTROL ASN.1 module from an H.245 PDF.

  extract_asn1.py <H.245.pdf> <out.asn>

The ASN.1 in the Recommendation's Annex A comes out of `pdftotext -layout`
with page furniture in it and with the "--" lost where the PDF wrapped a
comment onto a second line.  This repairs both mechanically and lists every
line it changes, then applies the few real defects in the 03/2022 text that
stop the module compiling (each one named below).  Nothing else is edited.

The output is a build intermediate, not committed: it is derived from a
copyrighted ITU text that already lives in `ITU Docs/`.
"""
import re
import subprocess
import sys

IDENT = r'[a-z][A-Za-z0-9-]*'
CODE = [
    r'^\s*\.\.\.,?\s*(--.*)?$',
    r'^\s*[{}][,\s]*(OPTIONAL)?[,\s]*(--.*)?$',
    r'^\s*\},?\s*(OPTIONAL|DEFAULT.*)?,?\s*(--.*)?$',
    r'^\s*' + IDENT + r'\s+(?:[A-Z]|\[)',
    r'^\s*' + IDENT + r'\s*\(\s*-?\d+\s*\)\s*,?\s*(--.*)?$',
    r'^\s*' + IDENT + r'\s*,\s*(--.*)?$',
    r'^\s*[A-Z][A-Za-z0-9-]*(\s*\{[^}]*\})?\s*::=',
    r'^\s*(OPTIONAL|DEFAULT|SIZE|\(|\)|\[|\]|BEGIN|END|IMPORTS|EXPORTS)',
    r'^\s*[A-Z][A-Za-z0-9-]*\s*(,|;)?\s*(--.*)?$',
    r'^\s*' + IDENT + r'\s*\([^)]*\)\s*,?\s*(--.*)?$',
    r'^\s*[a-z][A-Za-z0-9-]*\s+INTEGER\s*::=',
]


def is_code(s):
    return any(re.match(p, s) for p in CODE)


# Defects in the published text.  (anchor text on the line, old, new, why)
PATCHES = [
    ("sctpStreamID", "INTEGER (0..65535) OPTIONAL,", "INTEGER (0..65535),",
     "OPTIONAL is not allowed on a CHOICE alternative"),
    ("dataCapability              SEQUENCE OF DataCapability,",
     "SEQUENCE OF DataCapability,", "SEQUENCE OF DataApplicationCapability,",
     "DataCapability is referenced but never defined (read as DataApplicationCapability)"),
]


def main():
    pdf, out = sys.argv[1], sys.argv[2]
    text = subprocess.run(['pdftotext', '-layout', pdf, '-'], capture_output=True,
                          text=True, check=True).stdout.split('\n')
    start = next(i for i, l in enumerate(text) if l.startswith('MULTIMEDIA-SYSTEM-CONTROL {itu-t'))
    end = next(i for i in range(start, len(text)) if text[i].strip() == 'END')
    kept = []
    for l in text[start:end + 1]:
        s = l.replace('\f', '').rstrip()
        if re.match(r'^\s*(Rec\. ITU-T H\.245 \(\d\d/\d{4}\)\s+\d+|\d+\s+Rec\. ITU-T H\.245 \(\d\d/\d{4}\))\s*$', s):
            continue                                    # page footer
        kept.append(s)
    txt = '\n'.join(kept).replace('version(0)\n17 multimedia-system-control(0)}',
                                  'version(0) multimedia-system-control(0)}')
    lines = txt.split('\n')
    changed = []
    prev_comment = False
    for i, s in enumerate(lines):
        t = s.strip()
        if not t:
            continue
        if t.startswith('--'):
            prev_comment = True
            continue
        if prev_comment and not is_code(s):
            changed.append((i, s))
            lines[i] = ' ' * (len(s) - len(s.lstrip())) + '-- ' + t
            prev_comment = True
            continue
        prev_comment = '--' in s
    for i, s in enumerate(lines):
        if s and not s[0].isspace() and not re.search(r'OPTIONAL|DEFAULT', s) and not re.match(
                r'^(--|[A-Z][A-Za-z0-9-]*\s*(\{.*\})?\s*(::=.*)?$|[{}]|\.\.\.|BEGIN|END|IMPORTS|MULTIMEDIA-SYSTEM-CONTROL)', s):
            changed.append((i, s))
            lines[i] = '-- ' + s
    for anchor, old, new, why in PATCHES:
        hit = [i for i, s in enumerate(lines) if anchor in s and old in s]
        if len(hit) != 1:
            sys.exit('patch anchor %r matched %d lines (text changed?)' % (anchor, len(hit)))
        lines[hit[0]] = lines[hit[0]].replace(old, new)
        print('patched line %d: %s' % (hit[0] + 1, why), file=sys.stderr)
    for i, s in changed:
        print('commented wrapped-comment tail, line %d: %s' % (i + 1, s.strip()[:70]), file=sys.stderr)
    open(out, 'w').write('\n'.join(lines))


if __name__ == '__main__':
    main()
