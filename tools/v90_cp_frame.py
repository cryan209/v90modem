def excluded(x):
    if x<=17: return True
    if x in (34,51,68,85,102,119): return True
    return x>=136 and ((x-136)%17)==0
def getb(s,p,n):
    v=0
    for i in range(n): v|=int(s[p+i])<<i
    return v
DFI=[103,107,111,115,120,124]      # Table 14: start bit at 119 splits these
def parse(b,off):
    if off+300>len(b): return None
    s=b[off:]
    if not all(s[i] for i in range(17)) or s[17]: return None
    if s[34] or s[51] or s[68] or s[85] or s[102] or s[119]: return None
    idx=[getb(s,p,4) for p in DFI]
    if max(idx)>5: return None
    cc=max(idx)+1; differ=int(s[128])
    cs=136+136*cc*(2 if differ else 1)+1
    if off+cs+19>len(b): return None
    crc=0xFFFF
    for i in range(cs):
        if excluded(i): continue
        crc=(crc>>1)^0x8408 if ((s[i]^crc)&1) else crc>>1
    field=getb(s,cs,16)
    masks=[]
    for c in range(cc*(2 if differ else 1)):
        base=136+136*c; m=[]
        for ch in range(8):
            w=getb(s,base+1+17*ch,16)
            for k in range(16):
                if (w>>k)&1: m.append(ch*16+k)
        masks.append(m)
    return dict(off=off,cc=cc,differ=differ,flen=cs+19,
        kind=("CP" if s[19] else "CPt"),drn=getb(s,20,5),ack=int(s[33]),
        sil=int(s[30]),sr=getb(s,31,2),ld=getb(s,49,2),law=int(s[35]),
        trn1d=getb(s,52,16),dfi=idx,crc_ok=(crc==field),crc=crc,field=field,masks=masks)
def scan(b):
    out=[];run=0
    for i in range(len(b)):
        if b[i]: run+=1
        else:
            if run>=17:
                f=parse(b,i-run)
                if f: out.append(f)
            run=0
    return out

# ---------------------------------------------------------------------------
# A standalone Table-14 CP/CPt framer, for reading a recovered Phase 4 bit
# stream OUTSIDE the receiver (V90_CP_BIT_DUMP, or an offline demodulation).
#
# Frame length is NOT fixed: 8.5.2 makes it 292 + delta, where gamma =
# 136*(max constellation index in bits 103:127) and delta = 2*gamma+136 when
# bit 128 is set (the codec constellations differ) else gamma.  Six
# constellations with bit 128 set gives 292 + 1496 = 1788 bits, which is what
# the RasFinder sends and what its observed frame spacing measures exactly.
#
# TRAP: Table 14 puts a start bit at 119, splitting the six 4-bit constellation
# indices into 103/107/111/115 and 120/124.  Indexing them as 103+4*k reads
# intervals 4 and 5 off by one bit and yields nonsense (8, 10 instead of 4, 5).
#
# The CRC is 10.1.2.3.2/V.34 (crc_itu16_bits, init 0xFFFF, LSB-first, poly
# 0x8408) over the information bits with the frame sync, the start bits and
# every constellation-mask start bit EXCLUDED -- and a frame is valid when the
# computed information CRC EQUALS the 16-bit CRC field, not when the remainder
# over field+CRC is zero.  Mirrors vpcm_cp.c.
#
#   python3 -c "import sys;sys.path.insert(0,'tools');from v90_cp_frame import *; \
#     b=[1 if c=='1' else 0 for c in open('cp.bits').read() if c in '01']; \
#     [print(f) for f in scan(b)]"
