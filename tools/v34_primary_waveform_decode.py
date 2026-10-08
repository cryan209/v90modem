#!/usr/bin/env python3
"""Independent 3429/4800 V.34 HDX PCMU waveform -> ECM page decoder.

Scope: GPC, 16-state trellis, K=0, zero precoder, M=1; nonlinear projection
has constant magnitude here. No modem code/tables or transmit decisions are
used for acquisition. PP is generated from V.34 10.1.3.6; a fixed T/2 FIR
is trained ONLY on PP, with held-out PP validation. Primary decisions use
9.3.2/9.6, Table 13 and 7's GPC descrambler. HDLC/CRC validation is separate.
This is an offline receiver with noncausal supervised PP equalization, not
an emulation of the Eicon's acquisition implementation. numpy required;
Pillow required only for optional reference raster comparison.
"""
import argparse
import json
import re
import struct
from pathlib import Path
import numpy as np

BAUD = 24000/7
CARRIER = BAUD*4/7
LABELS = [[0,7,4,3],[5,2,1,6],[4,3,0,7],[1,6,5,2]]
TABLE13 = [[0,0,1,1,8,8,9,9],[3,2,2,3,11,10,10,11],
           [5,5,4,4,13,13,12,12],[6,7,7,6,14,15,15,14],
           [8,8,9,9,0,0,1,1],[11,10,10,11,3,2,2,3],
           [13,13,12,12,5,5,4,4],[14,15,15,14,6,7,7,6]]


def load_pcm(path, card):
    if card:
        return bytes(int(w[:2],16) for line in path.read_text().splitlines()
                     if '[*,1] SAMPLE[]' in line
                     for w in re.findall(r'\b[0-9A-F]{4}\b',line.split('SAMPLE[]')[1]))
    return path.read_bytes()


def demodulate(pcm, start, end, search_end):
    b=np.frombuffer(pcm,np.uint8).astype(np.int32); u=(~b)&255
    x=(((u&15)*8+132)<<((u>>4)&7))-132; x=np.where(u&128,-x,x)
    n=np.arange(len(x)); bb=x*np.exp(-2j*np.pi*CARRIER*n/8000)
    t=np.arange(161)-80; h=np.sinc(2*1900/8000*t)*np.hamming(161)
    bb=np.convolve(bb,h/h.sum(),'same')
    first=int(start*2*BAUD); last=min(int(end*2*BAUD),int((len(x)-32)*2*BAUD/8000))
    pos=np.arange(first,last)*8000/(2*BAUD); base=np.floor(pos).astype(int)
    y=np.zeros(len(pos),complex)
    for k in range(-12,13):
        y+=bb[base+k]*np.sinc(pos-base-k)*np.hamming(25)[k+12]
    pp=np.array([np.exp(1j*np.pi*((i//4)*(i%4)+(4 if (i//4)%3==1 else 0))/6)
                 for i in range(288)])
    limit=min(len(y),int((search_end-start)*2*BAUD)+576)
    corr=np.correlate(y[:limit],pp.repeat(2),'valid')
    peak=int(np.argmax(abs(corr))); best=None
    for off in range(max(16,peak-6),min(peak+7,len(y)-600)):
        ix=off+2*np.arange(288)[:,None]+np.arange(-16,17)[None,:]
        X=y[ix]; coef=np.linalg.lstsq(X[24:216],pp[24:216],rcond=1e-5)[0]
        error=float(np.mean(abs(X[216:272]@coef-pp[216:272])**2))
        if best is None or error<best[0]: best=(error,off,coef)
    if best is None or best[0]>.01:
        raise ValueError(f'PP acquisition failed: {None if best is None else best[0]}')
    error,off,coef=best
    # Fixed coefficients, no payload truth, adaptation, or fitting.
    count=(len(y)-off-18)//2-288
    r=[]
    for begin in range(0,count,8192):
        ix=off+2*np.arange(288+begin,288+min(count,begin+8192))[:,None]+np.arange(-16,17)[None,:]
        r.extend(y[ix]@coef)
    r=np.array(r); points=np.exp(1j*np.array([np.pi/4,-np.pi/4,-3*np.pi/4,3*np.pi/4]))
    rotations=np.argmin(abs(r[:,None]-points[None,:]),axis=1)
    return rotations,dict(pp_time_seconds=(first+off)/(2*BAUD),
                          pp_heldout_mse=error,symbols_decided=len(rotations))


def decode_bits(rot):
    state=prev=reg=0; inversion='0111011111111010'; vi=14; bits=[]
    invalid=[]
    for frame in range(len(rot)//8):
        high=((frame%15+1)*3)//15>((frame%15)*3)//15; nb=12 if high else 11
        for pair in range(4):
            a,b=map(int,rot[frame*8+pair*2:frame*8+pair*2+2]); v0=0
            if (4*(frame%15)+pair)%30==0:
                v0=int(inversion[vi%16]); vi+=1
            delta=(a-prev)%4; prev=a; parity=(b-a-((state&1)^v0))%4
            inputs=[parity//2,delta&1,delta>>1]; width=3 if pair<nb-8 else 2
            if parity not in (0,2) or (width==2 and inputs[2]!=0):
                invalid.append(len(bits))
            for bit in inputs[:width]:
                bits.append((bit^(reg>>17)^(reg>>22))&1)
                reg=((reg<<1)|bit)&((1<<23)-1)
            points=[(1+1j)*(-1j)**k for k in (a,b)]
            subs=[LABELS[((round(z.imag)+3)%8)//2][((round(z.real)+3)%8)//2] for z in points]
            u=TABLE13[subs[0]][subs[1]]; t1,t2,t3,t4=[(state>>j)&1 for j in range(4)]
            state=(t1<<3)|((t4^t1^((u>>1)&1))<<2)|((t3^((u>>1)&1))<<1)|(t2^(u&1))
        if frame==14: vi=0
    return ''.join(map(str,bits)),invalid


def hdlc(bits):
    flags=[m.start() for m in re.finditer('01111110',bits)]
    frames=[]; bad=aborts=rcp=0; stop=len(bits)
    for left,right in zip(flags,flags[1:]):
        raw=bits[left+8:right]
        if not raw: continue
        out=[]; ones=0; abort=False
        for c in raw:
            if c=='1':
                ones+=1
                if ones>=6: abort=True; break
                out.append(1)
            else:
                if ones!=5: out.append(0)
                ones=0
        if abort: aborts+=1; continue
        if len(out)<32: continue
        if len(out)%8: bad+=1; continue
        crc=0xffff
        for b in out: crc=(crc>>1)^(0x8408 if (crc^b)&1 else 0)
        if crc!=0xf0b8: bad+=1; continue
        data=bytes(sum(out[i+j]<<j for j in range(8)) for i in range(0,len(out),8))
        frames.append(data)
        if data[:3]==b'\xff\x03\x86':
            rcp+=1
            if rcp==3: stop=right+8; break
    return frames,stop,dict(valid_frames=len(frames),invalid_frames=bad,aborts=aborts,rcp_frames=rcp)


def tiff(payload,width,height):
    # T.4 MH bits are carried least-significant bit first in this AT+FBO=0 capture.
    tags=[(256,4,1,width),(257,4,1,height),(258,3,1,1),(259,3,1,3),
          (262,3,1,0),(266,3,1,2),(273,4,1,0),(278,4,1,height),
          (279,4,1,len(payload)),(292,4,1,0)]
    tags[6]=(273,4,1,8+2+12*len(tags)+4)
    return b'II'+struct.pack('<HIH',42,8,len(tags))+b''.join(struct.pack('<HHII',*t) for t in tags)+struct.pack('<I',0)+payload


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('waveform',type=Path); ap.add_argument('--card-rx',action='store_true')
    ap.add_argument('--start',type=float,default=10); ap.add_argument('--end',type=float,default=90)
    ap.add_argument('--search-end',type=float,default=18)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--reference-bits',type=Path)
    ap.add_argument('--reference-image',type=Path)
    args=ap.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    if not (0.01 <= args.start < args.search_end < args.end):
        ap.error('require 0.01 <= start < search-end < end')
    rotations,report=demodulate(load_pcm(args.waveform,args.card_rx),args.start,args.end,args.search_end)
    bits,invalid=decode_bits(rotations)
    frames,stop,ecm=hdlc(bits[168:]); stop+=168; bits=bits[:stop]
    report.update(ecm); report.update(b1_all_ones=bits[:168]=='1'*168,
        decoded_bits=len(bits),invalid_symbol_pairs=sum(i<stop for i in invalid))
    (args.output/'decoded.bits').write_text(bits)
    fcd={d[3]:d[4:-2] for d in frames if d[:3]==b'\xff\x03\x06'}
    report['image_frame_numbers']=sorted(fcd)
    report['image_frame_numbers_contiguous']=sorted(fcd)==list(range(len(fcd)))
    payload=b''.join(fcd[i] for i in sorted(fcd)); (args.output/'page.mh').write_bytes(payload)
    if args.reference_bits:
        truth=args.reference_bits.read_text().strip(); common=min(len(bits),len(truth))
        report['reference_bits_compared']=common
        report['reference_bit_errors']=sum(a!=b for a,b in zip(bits[:common],truth[:common]))
    if args.reference_image:
        from PIL import Image
        reference=Image.open(args.reference_image).convert('1'); width,height=reference.size
        image_path=args.output/'page.tif'; image_path.write_bytes(tiff(payload,width,height))
        received=Image.open(image_path).convert('1')
        report['raster_size']=[width,height]
        report['raster_different_pixels']=int(np.sum(np.array(received)!=np.array(reference)))
    (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    brief={k:v for k,v in report.items() if k!='image_frame_numbers'}
    brief['image_frames']=len(fcd); print(json.dumps(brief,indent=2))

if __name__=='__main__': main()
