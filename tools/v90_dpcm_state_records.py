#!/usr/bin/env python3
"""Decode the V.90 DPCM data pump's state records and its TrnProgress table.

Overlay `0x026a` is table driven in exactly the same shape as the INFO page
(`tools/info_state_records.py`), one level deeper: it runs **two** record
layers over the same two index tables.

    layer   base        terminator  state word           record pointer
    outer   DM(0x1fe9)  offset 0x17  DM(0x1ff7)          DM(0x120f)
    inner   DM(0x2001)  offset 0x10  DM(0x2008)          DM(0x204a)

PM 0x2fe3 is the applier for both -- the same `(offset, lo, hi)` triple walk
as INFO's PM 0x336a, with the base in MR0 and the terminator offset in MR1.
**TrnProgress is the outer layer's state word**: PM 0x2fba..0x2fbd publishes
`DM(0x3fc2) = DM(0x1ff7) AND 0x00ff`, and that single instruction is the only
writer of DM(0x3fc2) anywhere in the overlay.

Both layers translate their index fields through the same two tables:

    DM(0x0613), 0x40 entries  candidate index -> record address
    DM(0x05e0), 0x33 entries  condition index -> PM address

Condition index 0x00 is PM 0x3038, `AR = 0 + 1 ; RTS` -- constant false, the
"never" stub (INFO's PM 0x33c2).

**Records are deltas.** The applier writes only the offsets a record actually
carries, so every field it omits keeps the value the previous record left.
A record that sets some of its conditions to the never stub therefore does not
by itself prove the state is a dead end -- the slots it does not mention are
inherited, and whether those are live depends on the path taken to get there.
This tool reports what a record *sets*; use the replay
(`create_native_mips_modem()` and watch DM(0x120f)/DM(0x1ff7)) to establish
what a state's conditions actually are on a given path.

Usage:
    python3 tools/v90_dpcm_state_records.py                # the state map
    python3 tools/v90_dpcm_state_records.py --state 0xea   # one state
    python3 tools/v90_dpcm_state_records.py --record 0x1cce
    python3 tools/v90_dpcm_state_records.py --inner        # the inner layer
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from eicon_dsp_extract import load_sparse, parse_combifile

V90_DPCM_DOWNLOAD = 0x026A
COMBIFILE = Path("docs/firmware/dspdload.bin")

VECTOR_TABLE = 0x0613
VECTOR_TABLE_LEN = 0x40
CONDITION_TABLE = 0x05E0
CONDITION_TABLE_LEN = 0x33

# The record area, as laid out in the overlay's DM block.  Records are
# contiguous: a record that does not branch falls through to the next.
RECORD_AREA = (0x0F10, 0x1E00)

NEVER = 0x00            # condition index 0 -> PM 0x3038, constant false

# Conditions decoded so far.  PM 0x3010 is the dwell countdown -- the same
# shape as INFO's PM 0x3391: load the counter through I0, decrement, return
# the old value, so LE means "expired".  Unnamed indices still print their PM
# address.
CONDITIONS = {
    0x00: "never (AR = 0 + 1)",
    0x01: "outer dwell DM(0x1ff6) expired",
    0x02: "global countdown DM(0x20e0) expired",
    0x03: "inner dwell DM(0x2007) expired",
}


class Layer:
    """One record layer: base address, terminator offset, field names."""

    def __init__(self, name, base, end, state_offset, candidates, conditions):
        self.name = name
        self.base = base
        self.end = end
        self.state_offset = state_offset
        self.candidates = candidates      # first of four candidate offsets
        self.conditions = conditions      # first of five condition offsets

    def field(self, offset: int) -> str:
        if offset == self.state_offset:
            return "state"
        if self.candidates <= offset < self.candidates + 4:
            return f"next{offset - self.candidates}"
        if self.conditions <= offset < self.conditions + 4:
            return f"test{offset - self.conditions}"
        if offset == self.conditions + 4:
            return "pretest"
        return f"0x{self.base + offset:04x}"


OUTER = Layer("outer", 0x1FE9, 0x17, 0x0E, 0x0F, 0x13)
INNER = Layer("inner", 0x2001, 0x10, 0x07, 0x08, 0x0C)


def load_dm() -> list[int]:
    """Return the V.90 DPCM overlay's DM image as a sparse-filled list."""
    combifile = parse_combifile(COMBIFILE)
    for download in combifile["downloads"]:
        if download["download_id"] == V90_DPCM_DOWNLOAD and download["dm_blocks"]:
            image, _ = load_sparse(download["dm_blocks"], 0xFFFF, "DM")
            return image
    raise SystemExit(f"no download 0x{V90_DPCM_DOWNLOAD:04x} in {COMBIFILE}")


def decode(dm, address: int, layer: Layer,
           limit: int = 48) -> tuple[dict[int, int], int]:
    """Replay the layer's record applier; return it and the following address.

    The outer PM 0x2fe3 applier consumes the low byte of each stored word.
    The inner PM 0x2fee applier consumes the high byte instead. Both encode an
    (offset, value-low, value-high) triple, but they are not interchangeable.
    """
    out: dict[int, int] = {}
    for _ in range(limit):
        if layer is INNER:
            offset = (dm[address] >> 8) & 0xFF
            value = ((dm[address + 1] >> 8) & 0xFF) | (dm[address + 2] & 0xFF00)
        else:
            offset = dm[address] & 0xFF
            value = (dm[address + 1] & 0xFF) | ((dm[address + 2] & 0xFF) << 8)
        address += 3
        out[offset] = value
        if offset == layer.end:
            return out, address
    raise ValueError(f"record at 0x{address:04x} has no terminator")


def walk(dm, layer: Layer) -> list[tuple[int, dict[int, int], int]]:
    """Enumerate records reachable from the vector table.

    Records must not be found by scanning: a triple walk started at the wrong
    address still terminates on a byte that happens to equal the terminator
    offset, so a contiguous scan silently mis-aligns.  Every real record start
    is either a vector-table target or the fall-through of one, so seed from
    the table and follow fall-throughs.
    """
    seeds = [dm[VECTOR_TABLE + i] for i in range(VECTOR_TABLE_LEN)]
    pending = [a for a in seeds if RECORD_AREA[0] <= a < RECORD_AREA[1]]
    seen: dict[int, tuple[dict[int, int], int]] = {}
    while pending:
        address = pending.pop()
        if address in seen or not RECORD_AREA[0] <= address < RECORD_AREA[1]:
            continue
        try:
            record, following = decode(dm, address, layer)
        except ValueError:
            continue
        seen[address] = (record, following)
        pending.append(following)
        for slot in range(4):
            candidate = record.get(layer.candidates + slot)
            if candidate is not None and candidate < VECTOR_TABLE_LEN:
                pending.append(dm[VECTOR_TABLE + candidate])
    return [(a, r, f) for a, (r, f) in sorted(seen.items())]


def describe(dm, address: int, record: dict[int, int], following: int,
             layer: Layer) -> list[str]:
    fields = " ".join(
        f"{layer.field(offset)}={value:04x}"
        for offset, value in sorted(record.items())
        if offset < layer.candidates)
    lines = [f"@{address:04x}: {fields}   (falls through to @{following:04x})"]
    set_live = 0
    set_never = 0
    inherited = []
    pretest = record.get(layer.conditions + 4)
    if pretest is None:
        inherited.append("fall-through")
    else:
        pm = dm[CONDITION_TABLE + pretest]
        note = CONDITIONS.get(pretest)
        note = f"  ({note})" if note else ""
        lines.append(f"    fall-through: test[{pretest:02x}] -> PM {pm:04x}{note}")
        set_live += pretest != NEVER
        set_never += pretest == NEVER
    for slot in range(4):
        candidate = record.get(layer.candidates + slot)
        condition = record.get(layer.conditions + slot)
        if candidate is None and condition is None:
            inherited.append(f"slot {slot}")
            continue
        parts = [f"    slot {slot}:"]
        if candidate is not None:
            target = dm[VECTOR_TABLE + candidate]
            state = decode(dm, target, layer)[0].get(layer.state_offset)
            label = "----" if state is None else f"{state:04x}"
            parts.append(f"next[{candidate:02x}] -> @{target:04x} state {label}")
        if condition is not None:
            pm = dm[CONDITION_TABLE + condition]
            note = CONDITIONS.get(condition)
            note = f"  ({note})" if note else ""
            parts.append(f"test[{condition:02x}] -> PM {pm:04x}{note}")
            set_live += condition != NEVER
            set_never += condition == NEVER
        lines.append(" ".join(parts))
    if inherited:
        lines.append(f"    inherited (not set by this record): {', '.join(inherited)}")
    if set_live == 0 and set_never:
        lines.append("    every condition this record SETS is the never stub"
                     + (" -- dead end only if the inherited slots are too"
                        if inherited else " -- and it sets all of them: terminal"))
    return lines


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", type=lambda v: int(v, 0),
                    help="describe the record(s) publishing this TrnProgress")
    ap.add_argument("--record", type=lambda v: int(v, 0),
                    help="describe the record at this DM address")
    ap.add_argument("--inner", action="store_true",
                    help="use the inner layer (DM(0x2001), terminator 0x10)")
    args = ap.parse_args()

    dm = load_dm()
    layer = INNER if args.inner else OUTER

    if args.record is not None:
        record, following = decode(dm, args.record, layer)
        print("\n".join(describe(dm, args.record, record, following, layer)))
        return 0

    records = walk(dm, layer)
    if args.state is not None:
        hits = [r for r in records if r[1].get(layer.state_offset) == args.state]
        if not hits:
            print(f"no {layer.name} record publishes state 0x{args.state:04x}")
            return 1
        for address, record, following in hits:
            print("\n".join(describe(dm, address, record, following, layer)))
            print()
        # Which records can reach it, and under what condition.
        print(f"reached from:")
        for address, record, _ in records:
            for slot in range(4):
                candidate = record.get(layer.candidates + slot)
                if candidate is None:
                    continue
                target = dm[VECTOR_TABLE + candidate]
                if target not in {a for a, _, _ in hits}:
                    continue
                condition = record.get(layer.conditions + slot)
                pm = dm[CONDITION_TABLE + condition] if condition is not None else 0
                source = record.get(layer.state_offset)
                label = "----" if source is None else f"{source:04x}"
                print(f"  @{address:04x} state {label} slot {slot} "
                      f"test[{condition:02x}] -> PM {pm:04x}")
        return 0

    print(f"{len(records)} {layer.name} records in the V.90 DPCM overlay")
    states: dict[int, list[int]] = {}
    for address, record, _ in records:
        state = record.get(layer.state_offset)
        if state is not None:
            states.setdefault(state, []).append(address)
    print(f"{len(states)} distinct states:")
    for state in sorted(states):
        where = " ".join(f"@{a:04x}" for a in states[state])
        note = ""
        for address in states[state]:
            record, _ = decode(dm, address, layer)
            conditions = [record.get(layer.conditions + i) for i in range(5)]
            if all(c == NEVER for c in conditions):
                note = "   terminal (sets all five conditions to never)"
            elif all(c in (None, NEVER) for c in conditions):
                note = "   sets only never conditions; rest inherited"
        print(f"  {state:04x}  {where}{note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
