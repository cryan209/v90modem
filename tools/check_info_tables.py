#!/usr/bin/env python3
"""
Check decoded bitstreams against INFO field layouts used by this repo.

Input format defaults to the dumps produced by tools/dpsk_info_probe.py:
    0000:0063 1010...
"""

from __future__ import annotations

import argparse
from pathlib import Path


FSYNC_12 = "111101110010"  # fill(1111) + sync(01110010), bit 0 first
TABLE16_FRAME_BITS = 49
TAIL_FILL_4 = "1111"


def load_bits(path: Path) -> str:
    bits = []
    for line in path.read_text(encoding="ascii").splitlines():
        line = line.strip()
        if not line:
            continue
        if " " in line:
            bits.append(line.split(" ", 1)[1].strip())
        else:
            bits.append(line)
    return "".join(bits)


def find_all(haystack: str, needle: str) -> list[int]:
    out: list[int] = []
    pos = 0
    while True:
        idx = haystack.find(needle, pos)
        if idx < 0:
            return out
        out.append(idx)
        pos = idx + 1


def b2i_le(s: str) -> int:
    v = 0
    for i, ch in enumerate(s):
        if ch == "1":
            v |= 1 << i
    return v


def signed10(v: int) -> int:
    return -(v ^ 0x3FF) - 1 if (v & 0x200) else v


def decode_info0(bits: str, ext: bool) -> list[tuple[str, int, int, str, int]]:
    fields: list[tuple[str, int, int, str, int]] = []
    p = 0

    def take(name: str, n: int) -> None:
        nonlocal p
        s = bits[p:p + n]
        fields.append((name, p, p + n - 1, s, b2i_le(s)))
        p += n

    take("b12_support_2743", 1)
    take("b13_support_2800", 1)
    take("b14_support_3429", 1)
    take("b15_support_3000_low", 1)
    take("b16_support_3000_high", 1)
    take("b17_support_3200_low", 1)
    take("b18_support_3200_high", 1)
    take("b19_rate_3429_allowed", 1)
    take("b20_support_power_reduction", 1)
    take("b21_23_max_baud_rate_diff", 3)
    take("b24_from_cme_modem", 1)
    take("b25_support_1664_constellation", 1)
    take("b26_27_raw", 2)
    take("b28_ack", 1)
    if ext:
        take("b29_32_nominal_power_code", 4)
        take("b33_37_max_power_code", 5)
        take("b38_power_measured_at_codec_out", 1)
        take("b39_tx_clock_source", 1)
        take("b40_upstream_3429_support", 1)
        take("b41_reserved", 1)
    return fields


def decode_info1a(bits: str) -> list[tuple[str, int, int, str, int]]:
    fields: list[tuple[str, int, int, str, int]] = []
    p = 0

    def take(name: str, n: int) -> int:
        nonlocal p
        s = bits[p:p + n]
        v = b2i_le(s)
        fields.append((name, p, p + n - 1, s, v))
        p += n
        return v

    take("b12_17_raw", 6)
    take("b18_24_md", 7)
    take("b25_31_u_info", 7)
    take("b32_33_raw", 2)
    up = take("b34_36_upstream_symbol_rate_code", 3)
    down = take("b37_39_downstream_rate_code", 3)
    freq_raw = take("b40_49_freq_offset_raw", 10)
    fields.append(("b40_49_freq_offset_signed", -1, -1, "", signed10(freq_raw)))
    fields.append(("looks_v92_t18_pattern(up=6,down=6)", -1, -1, "", 1 if (up == 6 and down == 6) else 0))
    return fields


def decode_info1d(bits: str) -> list[tuple[str, int, int, str, int]]:
    fields: list[tuple[str, int, int, str, int]] = []
    p = 0

    def take(name: str, n: int) -> int:
        nonlocal p
        s = bits[p:p + n]
        v = b2i_le(s)
        fields.append((name, p, p + n - 1, s, v))
        p += n
        return v

    take("b12_14_power_reduction", 3)
    take("b15_17_additional_power_reduction", 3)
    take("b18_24_md", 7)
    for i in range(6):
        take(f"row{i}_use_high_carrier", 1)
        take(f"row{i}_pre_emphasis", 4)
        take(f"row{i}_max_bit_rate", 4)
    freq_raw = take("b79_88_freq_offset_raw", 10)
    fields.append(("b79_88_freq_offset_signed", -1, -1, "", signed10(freq_raw)))
    return fields


def crc8005_bits(bitstr: str) -> int:
    crc = 0xFFFF
    for ch in bitstr:
        bit = 1 if ch == "1" else 0
        if ((crc ^ bit) & 1) != 0:
            crc = (crc >> 1) ^ 0x8408
        else:
            crc >>= 1
        crc &= 0xFFFF
    return crc & 0xFFFF


def crc_match_info0d(payload46: str) -> bool:
    if len(payload46) < 46:
        return False
    data = payload46[:30]
    field = b2i_le(payload46[30:46])
    return field == crc8005_bits(data)


def crc_match_info1a(payload54: str) -> bool:
    if len(payload54) < 54:
        return False
    data = payload54[:38]
    field = b2i_le(payload54[38:54])
    return field == crc8005_bits(data)


def invert_bits(s: str) -> str:
    return "".join("1" if c == "0" else "0" for c in s)


def bits_to_str(v: int, n: int) -> str:
    return format(v, f"0{n}b")


def decode_table16_info0a(payload33: str) -> dict[str, int]:
    vals: dict[str, int] = {}
    vals["b12"] = b2i_le(payload33[0:1])
    vals["b13"] = b2i_le(payload33[1:2])
    vals["b14"] = b2i_le(payload33[2:3])
    vals["b15"] = b2i_le(payload33[3:4])
    vals["b16"] = b2i_le(payload33[4:5])
    vals["b17"] = b2i_le(payload33[5:6])
    vals["b18"] = b2i_le(payload33[6:7])
    vals["b19"] = b2i_le(payload33[7:8])
    vals["b20"] = b2i_le(payload33[8:9])
    vals["b21_23"] = b2i_le(payload33[9:12])
    vals["b24"] = b2i_le(payload33[12:13])
    vals["b25"] = b2i_le(payload33[13:14])
    vals["b26_v92_cap"] = b2i_le(payload33[14:15])
    vals["b27_short_p2_req"] = b2i_le(payload33[15:16])
    vals["b28_ack"] = b2i_le(payload33[16:17])
    vals["crc_field"] = b2i_le(payload33[17:33])
    vals["crc_calc"] = crc8005_bits(payload33[0:17])
    vals["crc_match"] = 1 if vals["crc_field"] == vals["crc_calc"] else 0
    return vals


def decode_table18_info1a(payload54: str) -> dict[str, int]:
    vals: dict[str, int] = {}
    vals["b12_13_num_filter_sections"] = b2i_le(payload54[0:2])
    vals["b14_15_ltot_code"] = b2i_le(payload54[2:4])
    vals["b16_17_lmax_code"] = b2i_le(payload54[4:6])
    vals["b18_24_md"] = b2i_le(payload54[6:13])
    vals["b25_31_uinfo"] = b2i_le(payload54[13:20])
    vals["b32_33_reserved"] = b2i_le(payload54[20:22])
    vals["b34_36_upstream_symbol_rate_code"] = b2i_le(payload54[22:25])
    vals["b37_39_downstream_rate_code"] = b2i_le(payload54[25:28])
    vals["b40_49_reserved"] = b2i_le(payload54[28:38])
    vals["crc_field"] = b2i_le(payload54[38:54])
    vals["crc_calc"] = crc8005_bits(payload54[0:38])
    vals["crc_match"] = 1 if vals["crc_field"] == vals["crc_calc"] else 0
    return vals


def classify_info1a(payload54: str) -> str:
    d = decode_table18_info1a(payload54)
    up = d["b34_36_upstream_symbol_rate_code"]
    down = d["b37_39_downstream_rate_code"]
    b32_33 = d["b32_33_reserved"]
    b40_49 = d["b40_49_reserved"]
    b12_17 = b2i_le(payload54[0:6])
    if up == 6 and down == 6 and b32_33 == 0 and b40_49 == 0x3FF:
        return "table18_pcm_upstream"
    if down == 6 and 3 <= up <= 5 and (b12_17 == 0) and (b2i_le(payload54[20:21]) == 0):
        return "table19_v34_upstream"
    return "unknown_or_misaligned"


def find_table16_full_frames(bits: str) -> list[tuple[int, str]]:
    out: list[tuple[int, str]] = []
    n = len(bits)
    for i in range(0, n - TABLE16_FRAME_BITS + 1):
        frame = bits[i:i + TABLE16_FRAME_BITS]
        if frame[:12] != FSYNC_12:
            continue
        if frame[45:49] != TAIL_FILL_4:
            continue
        out.append((i, frame))
    return out


def print_fields(payload_start: int, fields: list[tuple[str, int, int, str, int]]) -> None:
    for name, lo, hi, bits, val in fields:
        if lo >= 0:
            alo = payload_start + lo
            ahi = payload_start + hi
            print(f"    {name:40s} payload[{lo:02d}:{hi:02d}] abs[{alo}:{ahi}] bits={bits} val={val}")
        else:
            print(f"    {name:40s} val={val}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Check decoded bits against INFO field tables")
    ap.add_argument("dump", type=Path, help="Bit dump file from dpsk_info_probe.py")
    ap.add_argument("--slip-max", type=int, default=0, help="Try +/- this many bit slips per sync hit")
    ap.add_argument(
        "--sequence-only",
        action="store_true",
        help="Only print Table16 -> INFO1a sequence view (skip legacy all-table dump)",
    )
    args = ap.parse_args()

    bits = load_bits(args.dump)
    hits = find_all(bits, FSYNC_12)
    full49 = find_table16_full_frames(bits)
    print(f"file={args.dump}")
    print(f"total_bits={len(bits)} fsync12_hits={hits}")
    print(f"table16_full_49bit_matches={len(full49)}")
    print()

    if full49:
        print("=== Strict Table16 Full-Frame Matches (49 bits) ===")
        print()
        for start, frame in full49:
            payload33 = frame[12:45]
            t16 = decode_table16_info0a(payload33)
            print(f"frame_start={start} abs[{start}:{start + 48}]")
            print(f"  frame49={frame}")
            print(f"  fsync12={frame[:12]} payload33={payload33} tail4={frame[45:49]}")
            print(
                "  table16: "
                f"b26_v92_cap={t16['b26_v92_cap']} "
                f"b27_short_p2_req={t16['b27_short_p2_req']} "
                f"b28_ack={t16['b28_ack']} "
                f"crc_field=0x{t16['crc_field']:04X} "
                f"crc_calc=0x{t16['crc_calc']:04X} "
                f"crc_match={t16['crc_match']}"
            )
            print()

    print("=== Sequence decode (Table 16 first, then INFO1a/Table 18 or 19) ===")
    print()

    for i, hit in enumerate(hits):
        for slip in range(-args.slip_max, args.slip_max + 1):
            p0 = hit + len(FSYNC_12) + slip
            if p0 < 0:
                continue
            if p0 + 33 > len(bits):
                continue
            payload16 = bits[p0:p0 + 33]
            t16 = decode_table16_info0a(payload16)
            print(f"info0a sync@bit {hit} slip={slip:+d} payload_start={p0} abs[{p0}:{p0 + 32}]")
            print(f"  fsync12_abs[{hit}:{hit + len(FSYNC_12) - 1}]={bits[hit:hit + len(FSYNC_12)]}")
            print(f"  payload33={payload16}")
            print(f"  frame45={bits[hit:hit + len(FSYNC_12)]}{payload16}")
            print(
                "  table16: "
                f"b26_v92_cap={t16['b26_v92_cap']} "
                f"b27_short_p2_req={t16['b27_short_p2_req']} "
                f"b28_ack={t16['b28_ack']} "
                f"crc_field=0x{t16['crc_field']:04X} "
                f"crc_calc=0x{t16['crc_calc']:04X} "
                f"crc_match={t16['crc_match']}"
            )

            has_next = False
            for j in range(i + 1, len(hits)):
                hit2 = hits[j]
                for slip2 in range(-args.slip_max, args.slip_max + 1):
                    p1 = hit2 + len(FSYNC_12) + slip2
                    if p1 < 0 or p1 + 54 > len(bits):
                        continue
                    has_next = True
                    payload54 = bits[p1:p1 + 54]
                    cls = classify_info1a(payload54)
                    t18 = decode_table18_info1a(payload54)
                    eligible = (t16["b26_v92_cap"] == 1 and cls == "table18_pcm_upstream")
                    print(
                        f"  -> info1a sync@bit {hit2} slip={slip2:+d} "
                        f"payload_start={p1} abs[{p1}:{p1 + 53}] classify={cls} "
                        f"table18_allowed_after_table16={1 if eligible else 0}"
                    )
                    print(f"     fsync12_abs[{hit2}:{hit2 + len(FSYNC_12) - 1}]={bits[hit2:hit2 + len(FSYNC_12)]}")
                    print(f"     payload54={payload54}")
                    print(f"     frame66={bits[hit2:hit2 + len(FSYNC_12)]}{payload54}")
                    print(
                        "     fields: "
                        f"up_code={t18['b34_36_upstream_symbol_rate_code']} "
                        f"down_code={t18['b37_39_downstream_rate_code']} "
                        f"uinfo={t18['b25_31_uinfo']} "
                        f"b32_33={bits_to_str(t18['b32_33_reserved'], 2)} "
                        f"b40_49={bits_to_str(t18['b40_49_reserved'], 10)} "
                        f"crc_field=0x{t18['crc_field']:04X} "
                        f"crc_calc=0x{t18['crc_calc']:04X} "
                        f"crc_match={t18['crc_match']}"
                    )
            if not has_next:
                print("  -> no later sync hit available for INFO1a candidate in this dump")
            print()

    if args.sequence_only:
        return 0

    print("=== Legacy all-table candidate dump ===")
    print()

    for hit in hits:
        for slip in range(-args.slip_max, args.slip_max + 1):
            p0 = hit + len(FSYNC_12) + slip
            if p0 < 0:
                continue
            print(f"sync@bit {hit} slip={slip:+d} -> payload starts @ {p0}")

            if p0 + 33 <= len(bits):
                payload = bits[p0:p0 + 33]
                print("  INFO0a (33 payload bits) candidate")
                print_fields(p0, decode_info0(payload, ext=False))
                print(f"    trailing_bits_after_used={33 - 17}")

            if p0 + 46 <= len(bits):
                payload = bits[p0:p0 + 46]
                print("  INFO0d (46 payload bits) candidate")
                print_fields(p0, decode_info0(payload, ext=True))
                print(f"    crc8005_match={1 if crc_match_info0d(payload) else 0}")
                print(f"    crc8005_match_if_inverted_payload={1 if crc_match_info0d(invert_bits(payload)) else 0}")
                print(f"    trailing_bits_after_used={46 - 30}")

            if p0 + 54 <= len(bits):
                payload = bits[p0:p0 + 54]
                print("  INFO1a (54 payload bits) candidate")
                print_fields(p0, decode_info1a(payload))
                print(f"    crc8005_match={1 if crc_match_info1a(payload) else 0}")
                print(f"    crc8005_match_if_inverted_payload={1 if crc_match_info1a(invert_bits(payload)) else 0}")
                print(f"    trailing_bits_after_used={54 - 38}")

            if p0 + 93 <= len(bits):
                payload = bits[p0:p0 + 93]
                print("  INFO1d (93 payload bits) candidate")
                print_fields(p0, decode_info1d(payload))
                print(f"    trailing_bits_after_used={93 - 67}")

            print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
