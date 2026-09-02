#!/usr/bin/env python3
"""Read two usbmon pcaps side by side as SEQUENCES OF SCRIPT IDS.

Comparing raw hex by eye hides the thing that matters.  This identifies every
CD2_CONTROL_SCRIPT by matching its body against tools/hsf_scripts.py's
assembled output (tolerating patch bytes), and prints the vendor's control
sequence against ours with the bulk-stream start marked.  That is what showed
the probe had the 1/8/2 prelude and the 9/5 session start inverted.

  python3 tools/hsf_scripts.py <hsfusbcd2-i386.O> > /tmp/asm.txt
  python3 tools/hsf_seqdiff.py /tmp/asm.txt vendor.pcap ours.pcap
"""
import sys, struct, re
sys.path.insert(0,'tools')
from hsf_pcap import records

# assembled wire bodies, by script index
asm={}
txt=open(sys.argv[1]).read()
blocks=re.split(r"\n\[\s*(\d+)\]", txt)
for i in range(1,len(blocks),2):
    m=re.search(r"wire len=(\d+)\n((?:\s+[0-9a-f]{4}\s+[0-9a-f]+\n)+)",blocks[i+1])
    if m: asm[int(blocks[i])]="".join(re.findall(r"[0-9a-f]{4}\s+([0-9a-f]+)",m.group(2)))

def ident(body):
    h=body.hex()
    for k,v in asm.items():
        if v==h: return "script %d"%k
    for k,v in asm.items():
        if len(v)==len(h) and sum(a!=b for a,b in zip(v,h))<=6:
            return "script %d (patched)"%k
    return "UNKNOWN len=%d"%len(body)

def seq(path):
    out=[]
    for r in records(path):
        if r['xfer']==2 and r['setup'] and r['ev']=='S':
            b,q,v,i,l=struct.unpack('<BBHHH',r['setup'])
            if b==0x41 and q==0x02:
                out.append(("CONTROL_SCRIPT wValue=0x%04x wIndex=%d %s"%(v,i,ident(r['data'])), r['ts']))
            elif q==0x06: continue          # descriptor reads, host stack
            elif b==0xc0 and q==0x00: out.append(("CD2_GET_INFROMATION",r['ts']))
            elif b==0xc0: out.append(("CD2 vendor-IN req=0x%02x len=%d"%(q,l),r['ts']))
            elif b==0x02 and q==0x01: out.append(("CLEAR_FEATURE(HALT) ep 0x%02x"%i,r['ts']))
            elif b in (0x23,0xa3): continue  # hub/port, qemu side
            else: out.append(("rt=0x%02x req=0x%02x"%(b,q),r['ts']))
        if r['xfer']==3 and r['ev']=='S' and not (r['ep']&0x80):
            if not out or out[-1][0]!="<<< BULK OUT begins":
                out.append(("<<< BULK OUT begins",r['ts']))
    return out

for label,path in zip(("VENDOR","OURS"), sys.argv[2:4]):
    s=seq(path); t0=s[0][1]
    print("=== %s"%label)
    prev=None
    for name,ts in s:
        if name==prev and name!="<<< BULK OUT begins":
            continue
        print("  %7.3f  %s"%(ts-t0,name)); prev=name
    print()
