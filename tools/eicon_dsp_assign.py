#!/usr/bin/env python3
"""Locate Eicon MIPS DSP assignment routines and their TIKRNL mailboxes.

The protocol image is linked so that file-backed objects have runtime address
``0x80011000 + file_offset``.  This lets trace-format references identify the
otherwise stripped MIPS functions.  The TIKRNL mailbox symbols are recovered
from the combifile symbol table using the indices selected by ``dsp_assign``.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eicon_dsp_extract import FormatError, parse_combifile  # noqa: E402

RUNTIME_FILE_BIAS = 0x80011000

TRACE_LABELS = {
    "dsp_assign_fatal": b"dsp_assign fatal DSP trapped",
    "dsp_assign": b"dsp_assign %04x, %d, %d",
    "dsp_release": b"dsp_release",
    "dsp30_assign_fatal": b"dsp30_assign fatal DSP trapped",
    "dsp30_assign": b"dsp30_assign %04x, %d, %d",
    "dsp30_release": b"dsp30_release",
}

# te_dmlt.pm's download-id 0x0258 branch resolves these two symbols.  They are
# the host->TIKRNL command/database mailbox and the TIKRNL->host mailbox.
TIKRNL_WRITE_SYMBOL = 13
TIKRNL_READ_SYMBOL = 14

# Build 107-79 MIPS routines around the mailbox transaction.  These are file
# offsets; MIPS J/JAL targets in the image include RUNTIME_FILE_BIAS.
REQUEST_PARSER = (0x78138, 0x78388)
MAILBOX_SENDER = (0x786A4, 0x78CC0)
DATABASE_RECORD_APPEND = (0x7BBF8, 0x7BD0C)
DATABASE_RING_COMMIT = (0x7CD14, 0x7CF18)
COMMAND_SCRIPT_POINTERS = 0xEB248
COMMAND_SCRIPT_MODES = 2
COMMAND_SCRIPT_CODES = 79


def c_string(data: bytes, offset: int) -> str:
    end = data.find(b"\0", offset)
    if end < 0:
        end = len(data)
    return data[offset:end].decode("ascii", "replace")


def trace_xrefs(data: bytes, text: bytes) -> dict:
    match_offset = data.find(text)
    if match_offset < 0:
        raise ValueError(f"trace string not found: {text!r}")
    # Trace labels above deliberately omit the adapter/channel printf prefix.
    # MIPS code references the beginning of the containing C string.
    text_offset = data.rfind(b"\0", 0, match_offset) + 1
    runtime = RUNTIME_FILE_BIAS + text_offset
    hi = ((runtime + 0x8000) >> 16) & 0xFFFF
    lo = runtime & 0xFFFF
    references = []
    for reg in range(1, 32):
        lui = 0x3C000000 | (reg << 16) | hi
        addiu = 0x24000000 | (reg << 21) | (reg << 16) | lo
        for offset in range(0, len(data) - 4, 4):
            if struct.unpack_from("<I", data, offset)[0] != lui:
                continue
            for second in range(offset + 4, min(offset + 48, len(data)), 4):
                if struct.unpack_from("<I", data, second)[0] == addiu:
                    references.append(offset)
                    break
    return {
        "text": c_string(data, text_offset),
        "text_file_offset": text_offset,
        "runtime_address": runtime,
        "mips_xrefs": references,
    }


def symbol_address(download: dict, index: int) -> int:
    symbol = download["symbols"][index]
    segment = symbol["segment"]
    if segment < 4:
        return symbol["offset"]
    descriptor = next(s for s in download["segments"] if s["number"] == segment)
    return descriptor["base"] + symbol["offset"]


def dm_word(download: dict, address: int) -> int:
    for block in download["dm_blocks"]:
        if block.address <= address < block.address + block.words:
            return block.values[address - block.address]
    raise ValueError(f"TIKRNL DM address 0x{address:04x} is not populated")


def command_script_pointers(data: bytes) -> list[list[int | None]]:
    result: list[list[int | None]] = []
    for mode in range(COMMAND_SCRIPT_MODES):
        row = []
        for code in range(COMMAND_SCRIPT_CODES):
            runtime = struct.unpack_from(
                "<I",
                data,
                COMMAND_SCRIPT_POINTERS + 4 * (mode * COMMAND_SCRIPT_CODES + code),
            )[0]
            if runtime == 0:
                row.append(None)
            elif RUNTIME_FILE_BIAS <= runtime < RUNTIME_FILE_BIAS + len(data):
                row.append(runtime - RUNTIME_FILE_BIAS)
            else:
                # Codes >= 75 are rejected by the MIPS range check. Keeping an
                # out-of-image value visible catches table-boundary mistakes.
                row.append(runtime)
        result.append(row)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("protocol", type=Path, help="MIPS protocol image (te_dmlt.pm)")
    parser.add_argument("combifile", type=Path, help="DSP combifile (dspdload.bin)")
    parser.add_argument("--pretty", action="store_true", help="pretty-print JSON")
    args = parser.parse_args()

    try:
        protocol = args.protocol.read_bytes()
        trace = {name: trace_xrefs(protocol, text) for name, text in TRACE_LABELS.items()}
        combi = parse_combifile(args.combifile)
        matches = [
            d
            for d in combi["downloads"]
            if d["download_id"] == 0x0258
            and d["description"].startswith("TIKRNL81.F34 ")
        ]
        if len(matches) != 1:
            raise ValueError(f"expected one TIKRNL81.F34 download, found {len(matches)}")
        tikrnl = matches[0]
        write_address = symbol_address(tikrnl, TIKRNL_WRITE_SYMBOL)
        read_address = symbol_address(tikrnl, TIKRNL_READ_SYMBOL)
        result = {
            "protocol": str(args.protocol),
            "runtime_file_bias": RUNTIME_FILE_BIAS,
            "trace_xrefs": trace,
            "recovered_functions": {
                # Function starts are the nearest preceding MIPS stack-frame
                # prologues to the trace references in build 107-79.
                "dsp_assign": {"file_start": 0x79CC4, "file_end": 0x7B978},
                "dsp30_assign": {"file_start": 0x9775C, "file_end": 0x97DCC},
                "request_parser": {
                    "file_start": REQUEST_PARSER[0],
                    "file_end": REQUEST_PARSER[1],
                },
                "mailbox_sender": {
                    "file_start": MAILBOX_SENDER[0],
                    "file_end": MAILBOX_SENDER[1],
                },
                "database_record_append": {
                    "file_start": DATABASE_RECORD_APPEND[0],
                    "file_end": DATABASE_RECORD_APPEND[1],
                    "format_bytes": {"b": 1, "w": 2},
                },
                "database_ring_commit": {
                    "file_start": DATABASE_RING_COMMIT[0],
                    "file_end": DATABASE_RING_COMMIT[1],
                },
            },
            "command_script_pointer_table": {
                "file_offset": COMMAND_SCRIPT_POINTERS,
                "valid_code_limit": 75,
                "pointers_by_mode": command_script_pointers(protocol),
            },
            "tikrnl": {
                "download_id": tikrnl["download_id"],
                "description": tikrnl["description"],
                "write_mailbox": {
                    "symbol_index": TIKRNL_WRITE_SYMBOL,
                    "dm_address": write_address,
                    "words": tikrnl["symbols"][TIKRNL_WRITE_SYMBOL]["size"],
                    "producer_pointer_offset": 5,
                    "ring_start_offset": 8,
                    "ring_length_offset": 7,
                    "command_selector_offset": 0,
                },
                "read_mailbox": {
                    "symbol_index": TIKRNL_READ_SYMBOL,
                    "dm_address": read_address,
                    "words": tikrnl["symbols"][TIKRNL_READ_SYMBOL]["size"],
                    "control_offset": 0,
                },
                "initial_values": {
                    "write_ring_start": dm_word(tikrnl, write_address + 8),
                    "write_ring_words": dm_word(tikrnl, write_address + 7),
                    "write_producer": dm_word(tikrnl, write_address + 5),
                    "write_consumer": dm_word(tikrnl, write_address + 6),
                },
            },
        }
        print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    except (OSError, FormatError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
