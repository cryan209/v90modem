"""8051 disassembler, complete enough for the HSF script interpreter."""
SFR={0x80:"P0",0x81:"SP",0x82:"DPL",0x83:"DPH",0x87:"PCON",0x88:"TCON",0x89:"TMOD",
 0x8a:"TL0",0x8b:"TL1",0x8c:"TH0",0x8d:"TH1",0x90:"P1",0x98:"SCON",0x99:"SBUF",
 0xa0:"P2",0xa8:"IE",0xb0:"P3",0xb8:"IP",0xd0:"PSW",0xe0:"ACC",0xf0:"B"}
def sfr(a): return SFR.get(a, f"{a:02x}h")
ALU={0x2:"ADD",0x3:"ADDC",0x4:"ORL",0x5:"ANL",0x6:"XRL",0x9:"SUBB"}
def dis(d, pc, end):
    out=[]
    while pc<end and pc<len(d):
        s=pc; o=d[pc]; pc+=1
        def i8():
            nonlocal pc; v=d[pc]; pc+=1; return v
        def i16():
            nonlocal pc; v=(d[pc]<<8)|d[pc+1]; pc+=2; return v
        def rel():
            nonlocal pc; v=d[pc]; pc+=1; return (pc+(v-256 if v>127 else v))&0xffff
        hi,lo=o>>4,o&0xf
        t=None
        if o==0x00: t="NOP"
        elif o==0x02: t=f"LJMP {i16():04x}h"
        elif o==0x12: t=f"LCALL {i16():04x}h"
        elif o==0x22: t="RET"
        elif o==0x32: t="RETI"
        elif lo==1:
            a=i8(); tgt=((pc)&0xf800)|((hi>>1)<<8)|a
            t=(f"AJMP {tgt:04x}h" if hi%2==0 else f"ACALL {tgt:04x}h")
        elif o==0x80: t=f"SJMP {rel():04x}h"
        elif o==0x90: t=f"MOV DPTR,#{i16():04x}h"
        elif o==0x93: t="MOVC A,@A+DPTR"
        elif o==0x83: t="MOVC A,@A+PC"
        elif o==0xe0: t="MOVX A,@DPTR"
        elif o==0xf0: t="MOVX @DPTR,A"
        elif o in (0xe2,0xe3): t=f"MOVX A,@R{o-0xe2}"
        elif o in (0xf2,0xf3): t=f"MOVX @R{o-0xf2},A"
        elif o==0x73: t="JMP @A+DPTR"
        elif o==0x74: t=f"MOV A,#{i8():02x}h"
        elif o==0x75: a=i8(); t=f"MOV {sfr(a)},#{i8():02x}h"
        elif o in (0x76,0x77): t=f"MOV @R{o-0x76},#{i8():02x}h"
        elif 0x78<=o<=0x7f: t=f"MOV R{o-0x78},#{i8():02x}h"
        elif o==0xe4: t="CLR A"
        elif o==0xf4: t="CPL A"
        elif o==0xe5: t=f"MOV A,{sfr(i8())}"
        elif o==0xf5: t=f"MOV {sfr(i8())},A"
        elif o in (0xe6,0xe7): t=f"MOV A,@R{o-0xe6}"
        elif 0xe8<=o<=0xef: t=f"MOV A,R{o-0xe8}"
        elif o in (0xf6,0xf7): t=f"MOV @R{o-0xf6},A"
        elif 0xf8<=o<=0xff: t=f"MOV R{o-0xf8},A"
        elif o==0x85: a=i8(); t=f"MOV {sfr(i8())},{sfr(a)}"
        elif o in (0x86,0x87): t=f"MOV {sfr(i8())},@R{o-0x86}"
        elif 0x88<=o<=0x8f: t=f"MOV {sfr(i8())},R{o-0x88}"
        elif o in (0xa6,0xa7): t=f"MOV @R{o-0xa6},{sfr(i8())}"
        elif 0xa8<=o<=0xaf: t=f"MOV R{o-0xa8},{sfr(i8())}"
        elif o==0xc2: t=f"CLR {sfr(i8())}"
        elif o==0xd2: t=f"SETB {sfr(i8())}"
        elif o==0xb2: t=f"CPL {sfr(i8())}"
        elif o==0xa2: t=f"MOV C,{sfr(i8())}"
        elif o==0x92: t=f"MOV {sfr(i8())},C"
        elif o==0x72: t=f"ORL C,{sfr(i8())}"
        elif o==0x82: t=f"ANL C,{sfr(i8())}"
        elif o==0x20: b=i8(); t=f"JB {sfr(b)},{rel():04x}h"
        elif o==0x30: b=i8(); t=f"JNB {sfr(b)},{rel():04x}h"
        elif o==0x10: b=i8(); t=f"JBC {sfr(b)},{rel():04x}h"
        elif o==0x40: t=f"JC {rel():04x}h"
        elif o==0x50: t=f"JNC {rel():04x}h"
        elif o==0x60: t=f"JZ {rel():04x}h"
        elif o==0x70: t=f"JNZ {rel():04x}h"
        elif o==0xb4: v=i8(); t=f"CJNE A,#{v:02x}h,{rel():04x}h"
        elif o==0xb5: a=i8(); t=f"CJNE A,{sfr(a)},{rel():04x}h"
        elif o in (0xb6,0xb7): v=i8(); t=f"CJNE @R{o-0xb6},#{v:02x}h,{rel():04x}h"
        elif 0xb8<=o<=0xbf: v=i8(); t=f"CJNE R{o-0xb8},#{v:02x}h,{rel():04x}h"
        elif 0xd8<=o<=0xdf: t=f"DJNZ R{o-0xd8},{rel():04x}h"
        elif o==0xd5: a=i8(); t=f"DJNZ {sfr(a)},{rel():04x}h"
        elif o==0x04: t="INC A"
        elif o==0x14: t="DEC A"
        elif o==0x05: t=f"INC {sfr(i8())}"
        elif o==0x15: t=f"DEC {sfr(i8())}"
        elif o in (0x06,0x07): t=f"INC @R{o-6}"
        elif o in (0x16,0x17): t=f"DEC @R{o-0x16}"
        elif 0x08<=o<=0x0f: t=f"INC R{o-8}"
        elif 0x18<=o<=0x1f: t=f"DEC R{o-0x18}"
        elif o==0xa3: t="INC DPTR"
        elif o==0xa4: t="MUL AB"
        elif o==0x84: t="DIV AB"
        elif o==0xc0: t=f"PUSH {sfr(i8())}"
        elif o==0xd0: t=f"POP {sfr(i8())}"
        elif o==0xc3: t="CLR C"
        elif o==0xd3: t="SETB C"
        elif o==0xb3: t="CPL C"
        elif o==0xc4: t="SWAP A"
        elif o==0x23: t="RL A"
        elif o==0x03: t="RR A"
        elif o==0x33: t="RLC A"
        elif o==0x13: t="RRC A"
        elif o==0xd4: t="DA A"
        elif o in (0xc5,): t=f"XCH A,{sfr(i8())}"
        elif o in (0xc6,0xc7): t=f"XCH A,@R{o-0xc6}"
        elif 0xc8<=o<=0xcf: t=f"XCH A,R{o-0xc8}"
        elif hi in ALU:
            b=ALU[hi]
            if lo==4: t=f"{b} A,#{i8():02x}h"
            elif lo==5: t=f"{b} A,{sfr(i8())}"
            elif lo in (6,7): t=f"{b} A,@R{lo-6}"
            elif 8<=lo<=0xf: t=f"{b} A,R{lo-8}"
            elif lo==2: a=i8(); t=f"{b} {sfr(a)},A"
            elif lo==3: a=i8(); t=f"{b} {sfr(a)},#{i8():02x}h"
        if t is None: t=f"DB {o:02x}h"
        out.append((s, d[s:pc].hex(), t))
    return out
