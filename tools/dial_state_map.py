#!/usr/bin/env python3
"""Map DIAL's state machine by driving DM 0x3FB0 (the DIAL state word).

For each candidate state value, boot kernel+DIAL, poke the value into DM 0x3FB0,
enter the 0x1B9C dispatcher, and trace the conditional branches taken until it
either settles into a loop, goes idle, or runs past a budget. Record:
  - the sequence of conditional-jump decisions (taken/skipped, target)
  - DM 0x3FB0 final value (new state)
  - DM 0x3FB2/0x3FB3 vector pointers written (the next-action code addrs)
  - where it ends up (loop PC / idle)
"""
import ctypes, sys
from pathlib import Path

ADSP = ctypes.CDLL(str(Path(__file__).resolve().parent /
                       "adsp2181emu" / "libadsp2181.dylib"))
ADSP.adsp2181_create.restype = ctypes.c_void_p
for n, a in [('reset', [ctypes.c_void_p]), ('pm', [ctypes.c_void_p]),
             ('dm', [ctypes.c_void_p]), ('run', [ctypes.c_void_p, ctypes.c_int]),
             ('pc', [ctypes.c_void_p]), ('idle', [ctypes.c_void_p]),
             ('set_pc', [ctypes.c_void_p, ctypes.c_uint16])]:
    getattr(ADSP, 'adsp2181_' + n).argtypes = a
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_pc.restype = ctypes.c_uint16
ADSP.adsp2181_idle.restype = ctypes.c_int

COND = {0:'EQ',1:'NE',2:'GE',3:'LT',4:'GT',5:'LE',6:'AV',7:'NOT AV',
        8:'AC',9:'NOT AC',0xa:'AS',0xb:'NOT AS',0xc:'MV',0xd:'NOT MV',
        0xe:'CNTR>0',0xf:'TRUE'}


def rw(p):
    w = {}
    for line in Path(p).read_text().splitlines():
        f = line.split()
        if len(f) == 2:
            w[int(f[0], 16)] = int(f[1], 16)
    return w


def setup(kernel, dial):
    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)
    for a, v in rw(kernel + '/pm.words').items():
        pm[a] = v
    for a, v in rw(kernel + '/dm.words').items():
        dm[a] = v
    ADSP.adsp2181_run(cpu, 2000)            # boot kernel to IDLE
    for a, v in rw(dial + '/pm.words').items():
        pm[a] = v
    for a, v in rw(dial + '/dm.words').items():
        dm[a] = v
    return cpu, pm, dm


def trace_state(cpu, pm, dm, state_val, budget=400):
    dm[0x3FB0] = state_val & 0xffff
    ADSP.adsp2181_set_pc(cpu, 0x1B9C)
    decisions = []
    visited = {}
    last_loop = None
    for i in range(budget):
        pc = ADSP.adsp2181_pc(cpu)
        op = pm[pc] & 0xffffff
        visited[pc] = visited.get(pc, 0) + 1
        # detect a tight loop (3+ hits on one PC)
        if visited[pc] >= 3:
            last_loop = pc
            break
        # decode conditional jump/call (0x18-0x1f) and conditional alu/mac
        top = op >> 16
        taken = None
        note = ''
        if 0x18 <= top <= 0x1f:
            cond = op & 15
            tgt = (op >> 4) & 0x3fff
            kind = 'JUMP' if top <= 0x1b else 'CALL'
            # We can't read ASTAT directly; infer taken by PC after run
            ADSP.adsp2181_run(cpu, 1)
            npc = ADSP.adsp2181_pc(cpu)
            taken = (npc != pc + 1)
            note = f"{kind} {COND[cond]} ${tgt:04x} {'TAKEN' if taken else 'skip'}->${npc:04x}"
            decisions.append(note)
            continue
        if 0x20 <= top <= 0x27:  # conditional ALU/MAC
            cond = op & 15
            note = f"cond{'ALU' if top in (0x22,0x23,0x26,0x27) else 'MAC'} {COND[cond]}"
            decisions.append(note)
        ADSP.adsp2181_run(cpu, 1)
        if ADSP.adsp2181_idle(cpu):
            last_loop = 'IDLE'
            break
    final = ADSP.adsp2181_pc(cpu)
    return {
        'state_in': state_val,
        'state_out': dm[0x3FB0],
        'vec_3fb2': dm[0x3FB2],
        'vec_3fb3': dm[0x3FB3],
        'end_pc': final,
        'loop': last_loop,
        'n_steps': i + 1,
        'decisions': decisions,
    }


def main():
    repo = Path(__file__).resolve().parent.parent
    kernel = str(repo / 'artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel')
    dial = str(repo / 'artifacts/eicon-dsp/dial/0262-dial-fsk-fax.f34-overlay')
    cpu, pm, dm = setup(kernel, dial)
    # candidate states: the constants the ladder compares against, plus 0
    cands = [0x0000, 0x000B, 0x000F, 0x0010, 0x0004, 0x0011]
    # also sweep 0..0x20
    for s in list(range(0x00, 0x20)) + cands:
        s = s & 0xffff
        r = trace_state(cpu, pm, dm, s)
        print(f"state 0x{r['state_in']:04x} -> 0x{r['state_out']:04x}  "
              f"vec 3fb2=0x{r['vec_3fb2']:04x} 3fb3=0x{r['vec_3fb3']:04x}  "
              f"end=${r['end_pc']:04x} loop={r['loop']} steps={r['n_steps']}")
        for d in r['decisions'][:18]:
            print(f"    {d}")
        print()


if __name__ == '__main__':
    main()
