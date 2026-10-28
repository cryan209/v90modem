#!/usr/bin/env python3
"""Per-target table from one or more slm_targets.py batch directories.

  slm_targets_summary.py /root/slmx/auto [/root/slmx/v34 ...]
"""
import json
import re
import sys
from pathlib import Path


def rates(call_dir):
    """slmodemd's own final tx/rx bit rates and modulation for the call."""
    try:
        log = (call_dir / 'slm.log').read_text(errors='replace')
    except OSError:
        return '', ''
    m = re.findall(r'finally txbitrate (\d+),rxbitrate (\d+)', log)
    pcm = re.search(r'pcmType = (\w+)', log)
    mod = 'V.90' if pcm and 'drop to V34' not in log else 'V.34'
    if not m:
        return '', '-'
    tx, rx = m[-1]
    return 'up %s / down %s' % (tx, rx if rx != '0' else 'PCM'), mod


def grade(r):
    d = r.get('data') or {}
    t = r['target']
    if r.get('result') != 'CONNECT':
        return 'FAIL ' + str(r.get('result'))
    if t == 'ours':
        up, dn = d.get('up', {}), d.get('down', {})
        ok = up.get('lines') and dn.get('lines') and not up.get('missing') and not dn.get('missing')
        return '%s up %s/%s miss %s, down %s miss %s' % (
            'PASS' if ok else 'BAD', up.get('lines'), d.get('up_sent'), up.get('missing'),
            dn.get('lines'), dn.get('missing'))
    if t == '7900':
        ok = d.get('echo_sent') and d.get('echo_ok') == d.get('echo_sent')
        return '%s echo %s/%s' % ('PASS' if ok else 'BAD', d.get('echo_ok'), d.get('echo_sent'))
    if t == '7910':
        ok = d.get('opened') and d.get('echo_ok', 0) >= d.get('echo_sent', 0) - 1 and not d.get('fcs_bad')
        return '%s LCP echo %s/%s fcs_bad %s' % ('PASS' if ok else 'BAD', d.get('echo_ok'),
                                                 d.get('echo_sent'), d.get('fcs_bad'))
    ok = d.get('login') and d.get('probe_sent') and d.get('probe_ok') == d.get('probe_sent')
    return '%s login probes %s/%s' % ('PASS' if ok else 'BAD', d.get('probe_ok'), d.get('probe_sent'))


for batch in map(Path, sys.argv[1:]):
    calls = json.loads((batch / 'summary.json').read_text())
    print('==', batch)
    for r in calls:
        rt, mod = rates(batch / r['name'])
        drop = ''
        if r.get('dropped'):
            drop = 'drop@%ss' % r.get('held_s')
        print('%-10s %-5s %-6s %-15s t=%-5s %-26s %-9s %s' % (
            r['name'], mod, r.get('ms') or 'auto', r.get('connect', ''), r.get('t_connect', ''),
            rt, drop, grade(r)))
