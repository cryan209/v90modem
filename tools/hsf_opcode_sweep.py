#!/usr/bin/env python3
"""Sweep the HSF script opcode space looking for a register-read primitive.

Opcode 0x26 appends a byte to the completion notification, so a body of

    <op> <operands...> | 26 | 27 01 | 28 | 36

reports whatever the opcode left in the result byte.  The firmware's startup
does MOV f4h,#2dh / MOV f5h,#c0h, so an opcode that reads control register 0x2d
must come back with 0xC0 -- a ready-made oracle that owes nothing to guesswork.

Some opcodes WEDGE the device, and recovery is a physical replug (the firmware
is volatile, so a replug is a clean reset).  That is why this is resumable:
progress is written after every attempt, so a wedge costs one replug and no
repeated work.  Re-run with --resume after replugging.

    ./tools/hsf_opcode_sweep.py --wait 90     # catches the bootloader window
    ./tools/hsf_opcode_sweep.py --resume
"""
import argparse, json, os, re, subprocess, sys, time

STATE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "hsf_sweep_state.json")
PROBE = "./hsf_fxo_probe"
ORACLE_REG = 0x2D
ORACLE_VAL = "c0"
TAIL = "26" + "2701" + "28" + "36"


def load():
    try:
        return json.load(open(STATE))
    except Exception:
        return {"done": {}, "hits": [], "wedged_by": None}


def save(st):
    json.dump(st, open(STATE, "w"), indent=1)


def run(raw, secs=2):
    r = subprocess.run([PROBE, "--raw", raw, "--stream", str(secs)],
                       capture_output=True, text=True, timeout=60)
    o = r.stdout + r.stderr
    comp = [m.group(1) for m in
            (re.search(r"data=([0-9a-f]+)", l) for l in o.splitlines()
             if "data=" in l and "wValue=0x0001" in l) if m]
    alive = "NOT RESPONDING" not in o and "GET_INFROMATION failed" not in o
    return comp, alive


def wait_window(secs):
    print(f"waiting up to {secs}s for the bootloader window -- REPLUG NOW", flush=True)
    r = subprocess.run([PROBE, "--wait", str(secs), "--load"],
                       capture_output=True, text=True, timeout=secs + 60)
    print((r.stdout + r.stderr).strip(), flush=True)
    return "firmware running" in (r.stdout + r.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=int, default=0)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--nops", type=int, default=2, help="operand bytes to try (1..n)")
    a = ap.parse_args()

    if a.wait and not wait_window(a.wait):
        sys.exit("the window never opened")

    st = load() if a.resume or os.path.exists(STATE) else {"done": {}, "hits": [],
                                                           "wedged_by": None}
    todo = [(op, n) for op in range(0x69) for n in range(1, a.nops + 1)
            if f"{op:02x}/{n}" not in st["done"]]
    print(f"{len(todo)} combinations left of {0x69 * a.nops}", flush=True)

    for op, n in todo:
        key = f"{op:02x}/{n}"
        operands = f"{ORACLE_REG:02x}" + "00" * (n - 1)
        raw = f"{op:02x}{operands}{TAIL}"
        try:
            comp, alive = run(raw)
        except subprocess.TimeoutExpired:
            comp, alive = [], False
        st["done"][key] = {"comp": comp, "alive": alive}
        hit = any(ORACLE_VAL in c for c in comp)
        if hit:
            st["hits"].append(key)
        if not alive:
            st["wedged_by"] = key
            save(st)
            print(f"op 0x{op:02x} ({n} operand) WEDGED THE DEVICE -- replug and "
                  f"re-run with --wait N --resume", flush=True)
            return
        save(st)
        print(f"op 0x{op:02x} n={n}: {str(comp):30s}"
              f"{'   <<< ORACLE 0xC0' if hit else ''}", flush=True)
        time.sleep(0.4)

    print("\nsweep complete.  hits:", st["hits"], flush=True)


if __name__ == "__main__":
    main()
