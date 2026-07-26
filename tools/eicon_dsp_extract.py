#!/usr/bin/env python3
"""Extract addressed ADSP-218x PM/DM images from Eicon DSP combifiles.

The container structures are the Eicon ``t_dsp_*`` structures published in
Linux's historical drivers/isdn/hardware/eicon/dsp_defs.h.  The modem firmware
uses little-endian fields.  PM blocks in the V.34/V.90 overlays carry one
24-bit ADSP instruction in each little-endian 32-bit container.

Output images are sparse address-space images:
  dm.bin       65536 little-endian 16-bit words (gaps filled with zero)
  pm.bin       65536 packed little-endian 24-bit words (gaps filled with zero)
  dm.words     address/value text map containing only loaded words
  pm.words     address/value text map containing only loaded words
  metadata.json

The text maps and metadata distinguish a loaded zero from an unpopulated gap.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

COMBI_MAGIC = b"Eicon.Diehl DSP Download Combifile".ljust(48, b"\0")
FILE_MAGIC = b"Eicon.Diehl DSP Download\0File\0".ljust(48, b"\0")
FORMAT_VERSION = 0x0100
FIRST_RELOCATABLE_SEGMENT = 4
DATA_BLOCK_PM = 0x0001
DATA_BLOCK_DWORD = 0x0002
DATA_BLOCK_RESOLVE = 0x0004

COMBI_HEADER = struct.Struct("<48s7H")
FILE_HEADER = struct.Struct("<48s17H")
MEMORY_BLOCK = struct.Struct("<4H")
SEGMENT = struct.Struct("<5H")
SYMBOL = struct.Struct("<4H")
DATA_BLOCK = struct.Struct("<4H")


class FormatError(ValueError):
    pass


@dataclass
class SegmentDesc:
    number: int
    memory_block: int
    attributes: int
    base: int
    size: int
    alignment: int


@dataclass
class DataBlock:
    domain: str
    attributes: int
    segment: int
    offset: int
    address: int
    words: int
    file_offset: int
    values: list[int]
    relocations: list[dict]


def need(data: bytes, pos: int, size: int, what: str) -> None:
    if pos < 0 or size < 0 or pos + size > len(data):
        raise FormatError(f"truncated {what} at file offset 0x{pos:x}")


def read_struct(fmt: struct.Struct, data: bytes, pos: int, what: str) -> tuple:
    need(data, pos, fmt.size, what)
    return fmt.unpack_from(data, pos)


def c_string(raw: bytes) -> str:
    return raw.split(b"\0", 1)[0].decode("latin-1", errors="replace")


def slugify(text: str, download_id: int) -> str:
    text = re.sub(r"\s+Version.*$", "", text, flags=re.IGNORECASE)
    text = re.sub(r"[^A-Za-z0-9._-]+", "-", text).strip("-.").lower()
    return f"{download_id:04x}-{text or 'download'}"


def segment_address(segment: int, offset: int, segments: list[SegmentDesc]) -> int:
    if segment < FIRST_RELOCATABLE_SEGMENT:
        return offset
    idx = segment - FIRST_RELOCATABLE_SEGMENT
    if idx >= len(segments):
        raise FormatError(
            f"data block references relocatable segment {segment}, "
            f"but only {len(segments)} segment descriptors exist"
        )
    return segments[idx].base + offset


def relocation_base(segment: int, segments: list[SegmentDesc]) -> int:
    if segment < FIRST_RELOCATABLE_SEGMENT:
        return 0
    idx = segment - FIRST_RELOCATABLE_SEGMENT
    if idx >= len(segments):
        raise FormatError(f"relocation references missing segment {segment}")
    return segments[idx].base


def resolve_block_values(
    raw_values: list[int],
    attributes: int,
    domain: str,
    segments: list[SegmentDesc],
    block_index: int,
) -> tuple[list[int], list[dict]]:
    mask = 0xFFFF if domain == "dm" else 0xFFFFFF
    values: list[int] = []
    relocations: list[dict] = []
    for word_index, raw in enumerate(raw_values):
        relocation = (raw >> 24) & 0xFF
        value = raw & mask
        extra = raw & ~(mask | 0xFF000000)
        if extra:
            raise FormatError(
                f"{domain} block {block_index} word {word_index} has "
                f"unsupported container bits 0x{extra:x}"
            )
        if relocation:
            if not attributes & DATA_BLOCK_RESOLVE:
                raise FormatError(
                    f"{domain} block {block_index} has relocation byte "
                    f"0x{relocation:02x} without the RESOLVE attribute"
                )
            target_segment = relocation & 0x3F
            relocation_type = relocation & 0xC0
            base = relocation_base(target_segment, segments)
            before = value
            resolved = False
            if relocation_type == 0x00:
                # Recovered from the shipping MIPS loader in te_dmlt.pm,
                # build 117-926, at file offsets 0x75e64..0x75e7c.
                value = value + (base if domain == "dm" else base << 8)
                resolved = True
            elif relocation_type == 0x40:
                # Address in the low part of a PM data word (0x75e80..94).
                value += base
                resolved = True
            elif relocation_type == 0x80:
                # Address in a standard ADSP command (0x75e98..0x75eac).
                value += base << 4
                resolved = True
            elif relocation_type == 0xC0:
                # CALL/JUMP-on-FLAG_IN permutes its split address around a
                # base<<2 addition (te_dmlt.pm 0x75eb0..0x75efc).
                temporary = (
                    (value & 0xFF0003)
                    | ((value & 0x00FFF0) >> 2)
                    | ((value & 0x00000C) << 12)
                )
                temporary += base << 2
                value = (
                    (temporary & 0xFF0003)
                    | ((temporary & 0x003FFC) << 2)
                    | ((temporary & 0x00C000) >> 12)
                )
                resolved = True
            value &= mask
            relocations.append(
                {
                    "word_index": word_index,
                    "code": relocation,
                    "type": relocation_type >> 6,
                    "target_segment": target_segment,
                    "target_base": base,
                    "before": before,
                    "after": value,
                    "resolved": resolved,
                }
            )
        values.append(value)
    return values, relocations


def parse_blocks(
    data: bytes,
    pos: int,
    section_size: int,
    count: int,
    expected_domain: str,
    segments: list[SegmentDesc],
) -> tuple[list[DataBlock], int]:
    end = pos + section_size
    need(data, pos, section_size, f"{expected_domain} data section")
    blocks: list[DataBlock] = []

    for index in range(count):
        header_pos = pos
        attributes, segment, offset, words = read_struct(
            DATA_BLOCK, data, pos, f"{expected_domain} block {index} header"
        )
        pos += DATA_BLOCK.size
        domain = "pm" if attributes & DATA_BLOCK_PM else "dm"
        if domain != expected_domain:
            raise FormatError(
                f"{expected_domain} section block {index} is marked {domain} "
                f"(attributes 0x{attributes:04x})"
            )
        if domain == "dm":
            width = 4 if attributes & DATA_BLOCK_DWORD else 2
        else:
            # All V.34/V.90 overlays examined use DWORD PM containers.  The
            # non-DWORD form is packed 24-bit PM and padded to a word boundary.
            width = 4 if attributes & DATA_BLOCK_DWORD else 3
        byte_count = words * width
        padded_count = byte_count + (byte_count & 1)
        need(data, pos, padded_count, f"{domain} block {index} payload")

        if width == 2:
            values = list(struct.unpack_from(f"<{words}H", data, pos))
        elif width == 4:
            containers = list(struct.unpack_from(f"<{words}I", data, pos))
            if domain == "pm":
                # Eicon stores DWORD PM as two host words: instruction bits
                # 23:8 in the first word, instruction bits 7:0 in the low byte
                # of the second, and relocation in its high byte.  This is the
                # exact unpacking at te_dmlt.pm 0x75e04..0x75e24.
                values = [
                    (raw & 0xFF000000)
                    | ((raw & 0x0000FFFF) << 8)
                    | ((raw >> 16) & 0xFF)
                    for raw in containers
                ]
            else:
                values = containers
        else:
            values = [
                int.from_bytes(data[pos + 3 * i : pos + 3 * i + 3], "little")
                for i in range(words)
            ]
        values, relocations = resolve_block_values(
            values, attributes, domain, segments, index
        )
        address = segment_address(segment, offset, segments)
        if address + words > 0x10000:
            raise FormatError(
                f"{domain} block {index} exceeds the 16-bit ADSP address space: "
                f"0x{address:04x}+0x{words:x}"
            )
        blocks.append(
            DataBlock(
                domain=domain,
                attributes=attributes,
                segment=segment,
                offset=offset,
                address=address,
                words=words,
                file_offset=header_pos,
                values=values,
                relocations=relocations,
            )
        )
        pos += padded_count

    if pos != end:
        raise FormatError(
            f"{expected_domain} section length mismatch: parsed to 0x{pos:x}, "
            f"declared end is 0x{end:x}"
        )
    return blocks, pos


def parse_download(data: bytes, pos: int, usage_mask: bytes, index: int) -> tuple[dict, int]:
    start = pos
    fields = read_struct(FILE_HEADER, data, pos, f"download {index} header")
    magic = fields[0]
    values = fields[1:]
    if magic != FILE_MAGIC:
        raise FormatError(f"invalid download magic at file offset 0x{pos:x}")
    (
        version,
        download_id,
        flags,
        processing_power,
        interface_channels,
        header_size,
        description_size,
        memory_table_size,
        memory_count,
        segment_table_size,
        segment_count,
        symbol_table_size,
        symbol_count,
        dm_size,
        dm_count,
        pm_size,
        pm_count,
    ) = values
    if version != FORMAT_VERSION:
        raise FormatError(f"unsupported download version 0x{version:04x}")
    if header_size < FILE_HEADER.size:
        raise FormatError(f"download header size {header_size} is too small")
    pos += FILE_HEADER.size
    need(data, pos, header_size - FILE_HEADER.size, "excess download header")
    excess_header = data[pos : pos + header_size - FILE_HEADER.size]
    pos += header_size - FILE_HEADER.size

    need(data, pos, description_size, "download description")
    description_raw = data[pos : pos + description_size]
    description = c_string(description_raw)
    pos += description_size

    if memory_table_size != memory_count * MEMORY_BLOCK.size:
        raise FormatError("memory-block table size/count mismatch")
    memory_blocks = []
    for n in range(memory_count):
        alias, mem_type, address, size = read_struct(
            MEMORY_BLOCK, data, pos, f"memory block {n}"
        )
        memory_blocks.append(
            {"number": n, "alias": alias, "type": mem_type, "address": address, "size": size}
        )
        pos += MEMORY_BLOCK.size

    if segment_table_size != segment_count * SEGMENT.size:
        raise FormatError("segment table size/count mismatch")
    segments = []
    for n in range(segment_count):
        mem, attrs, base, size, alignment = read_struct(SEGMENT, data, pos, f"segment {n}")
        segments.append(
            SegmentDesc(FIRST_RELOCATABLE_SEGMENT + n, mem, attrs, base, size, alignment)
        )
        pos += SEGMENT.size

    if symbol_table_size != symbol_count * SYMBOL.size:
        raise FormatError("symbol table size/count mismatch")
    symbols = []
    for n in range(symbol_count):
        symbol_id, segment, offset, size = read_struct(SYMBOL, data, pos, f"symbol {n}")
        symbols.append(
            {"id": symbol_id, "segment": segment, "offset": offset, "size": size}
        )
        pos += SYMBOL.size

    dm_blocks, pos = parse_blocks(data, pos, dm_size, dm_count, "dm", segments)
    pm_blocks, pos = parse_blocks(data, pos, pm_size, pm_count, "pm", segments)
    raw = data[start:pos]
    result = {
        "index": index,
        "file_offset": start,
        "file_size": pos - start,
        "sha256": hashlib.sha256(raw).hexdigest(),
        "usage_mask": usage_mask.hex(),
        "description": description,
        "download_id": download_id,
        "flags": flags,
        "required_processing_power": processing_power,
        "interface_channels": interface_channels,
        "excess_header": excess_header.hex(),
        "memory_blocks": memory_blocks,
        "segments": [asdict(x) for x in segments],
        "symbols": symbols,
        "dm_blocks": dm_blocks,
        "pm_blocks": pm_blocks,
        "raw": raw,
    }
    return result, pos


def parse_combifile(path: Path) -> dict:
    data = path.read_bytes()
    fields = read_struct(COMBI_HEADER, data, 0, "combifile header")
    magic = fields[0]
    if magic != COMBI_MAGIC:
        raise FormatError("not an Eicon DSP Download Combifile")
    (
        version,
        header_size,
        description_size,
        directory_entries,
        directory_size,
        download_count,
        usage_mask_size,
    ) = fields[1:]
    if version != FORMAT_VERSION:
        raise FormatError(f"unsupported combifile version 0x{version:04x}")
    if header_size < COMBI_HEADER.size:
        raise FormatError(f"combifile header size {header_size} is too small")
    pos = header_size
    need(data, pos, description_size, "combifile description")
    description = c_string(data[pos : pos + description_size])
    pos += description_size

    if directory_size < directory_entries * 4:
        raise FormatError("combifile directory is shorter than its entry count")
    need(data, pos, directory_size, "combifile directory")
    directory = []
    for n in range(directory_entries):
        card_type, file_set = struct.unpack_from("<2H", data, pos + 4 * n)
        directory.append({"card_type": card_type, "file_set": file_set})
    pos += directory_size

    downloads = []
    for index in range(download_count):
        need(data, pos, usage_mask_size, f"download {index} usage mask")
        usage_mask = data[pos : pos + usage_mask_size]
        pos += usage_mask_size
        download, pos = parse_download(data, pos, usage_mask, index)
        downloads.append(download)
    if pos != len(data):
        raise FormatError(
            f"combifile has {len(data) - pos} trailing bytes after declared downloads"
        )
    return {
        "path": str(path),
        "size": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
        "description": description,
        "usage_mask_size": usage_mask_size,
        "directory": directory,
        "downloads": downloads,
    }


def load_sparse(blocks: Iterable[DataBlock], mask: int, domain: str) -> tuple[list[int], set[int]]:
    image = [0] * 0x10000
    loaded: set[int] = set()
    for block in blocks:
        for i, raw_value in enumerate(block.values):
            address = block.address + i
            value = raw_value & mask
            if raw_value & ~mask:
                raise FormatError(
                    f"{domain} value 0x{raw_value:x} at 0x{address:04x} "
                    f"exceeds the target word width"
                )
            if address in loaded and image[address] != value:
                raise FormatError(
                    f"conflicting {domain} values at address 0x{address:04x}: "
                    f"0x{image[address]:x} and 0x{value:x}"
                )
            image[address] = value
            loaded.add(address)
    return image, loaded


def serializable_download(download: dict) -> dict:
    result = {k: v for k, v in download.items() if k not in {"raw", "dm_blocks", "pm_blocks"}}
    for key in ("dm_blocks", "pm_blocks"):
        result[key] = [
            {k: v for k, v in asdict(block).items() if k != "values"}
            for block in download[key]
        ]
    return result


def write_download(download: dict, output_root: Path) -> Path:
    unresolved = [
        relocation
        for key in ("dm_blocks", "pm_blocks")
        for block in download[key]
        for relocation in block.relocations
        if not relocation["resolved"]
    ]
    if unresolved:
        kinds = ", ".join(
            sorted({f"type-{relocation['type']}" for relocation in unresolved})
        )
        raise FormatError(
            f"{download['description']} contains {len(unresolved)} unsupported "
            f"relocations ({kinds}); refusing to emit a misleading image"
        )
    target = output_root / slugify(download["description"], download["download_id"])
    target.mkdir(parents=True, exist_ok=True)
    (target / "download.bin").write_bytes(download["raw"])

    dm, dm_loaded = load_sparse(download["dm_blocks"], 0xFFFF, "DM")
    pm, pm_loaded = load_sparse(download["pm_blocks"], 0xFFFFFF, "PM")
    (target / "dm.bin").write_bytes(struct.pack("<65536H", *dm))
    pm_bytes = bytearray()
    for value in pm:
        pm_bytes += value.to_bytes(3, "little")
    (target / "pm.bin").write_bytes(pm_bytes)
    (target / "dm.words").write_text(
        "".join(f"{address:04x} {dm[address]:04x}\n" for address in sorted(dm_loaded))
    )
    (target / "pm.words").write_text(
        "".join(f"{address:04x} {pm[address]:06x}\n" for address in sorted(pm_loaded))
    )
    metadata = serializable_download(download)
    metadata["loaded_dm_words"] = len(dm_loaded)
    metadata["loaded_pm_words"] = len(pm_loaded)
    (target / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return target


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("combifile", type=Path, help="Eicon dspdload/dspdvmdm combifile")
    parser.add_argument("-o", "--output", type=Path, default=Path("artifacts/eicon-dsp"))
    parser.add_argument(
        "--match",
        default=r"V\.34 Overlay|V\.90 (?:DPCM|APCM) Overlay",
        help="case-insensitive regular expression selecting descriptions",
    )
    parser.add_argument(
        "--card-type",
        type=lambda value: int(value, 0),
        help="select only downloads required by this Eicon card-type number",
    )
    parser.add_argument("--list", action="store_true", help="list downloads without extracting")
    args = parser.parse_args()

    try:
        combi = parse_combifile(args.combifile)
        pattern = re.compile(args.match, re.IGNORECASE)
        file_set = None
        if args.card_type is not None:
            matches = [
                entry["file_set"]
                for entry in combi["directory"]
                if entry["card_type"] == args.card_type
            ]
            if not matches:
                raise FormatError(f"card type {args.card_type} is not in the combifile directory")
            file_set = matches[0]
            if file_set // 8 >= combi["usage_mask_size"]:
                raise FormatError(
                    f"card type {args.card_type} maps to invalid file set {file_set}"
                )

        def required(download: dict) -> bool:
            if file_set is None:
                return True
            mask = bytes.fromhex(download["usage_mask"])
            return bool(mask[file_set // 8] & (1 << (file_set & 7)))

        selected = [
            d
            for d in combi["downloads"]
            if required(d) and pattern.search(d["description"])
        ]
        print(f"{args.combifile}: {combi['description']}")
        card_text = (
            f", card_type={args.card_type}, file_set={file_set}"
            if args.card_type is not None
            else ""
        )
        print(f"downloads={len(combi['downloads'])}, selected={len(selected)}{card_text}")
        shown = combi["downloads"] if args.list and file_set is None else (
            [d for d in combi["downloads"] if required(d)] if args.list else selected
        )
        for download in shown:
            line = (
                f"[{download['index']:3d}] id=0x{download['download_id']:04x} "
                f"flags=0x{download['flags']:02x} "
                f"offset=0x{download['file_offset']:x} size={download['file_size']:7d} "
                f"DM={sum(b.words for b in download['dm_blocks']):5d} "
                f"PM={sum(b.words for b in download['pm_blocks']):5d}  "
                f"{download['description']}"
            )
            if args.list:
                print(line)
            else:
                target = write_download(download, args.output)
                print(f"{line} -> {target}")
        if not args.list and not selected:
            print(f"no descriptions matched {args.match!r}", file=sys.stderr)
            return 1
    except (OSError, FormatError, re.error) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
