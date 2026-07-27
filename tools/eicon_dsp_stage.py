#!/usr/bin/env python3
"""Build the DSP code image the MIPS protocol firmware expects in card RAM.

On a real card the host driver stages a DSP download image at
`DspCodeBaseAddr` before releasing the MIPS, and writes that address into
the protocol image header at `OFFS_DSP_CODE_BASE_ADDR` (0x6c).  The MIPS
entry (`0x80082f90` in te_dmlt.pm) reads the address from the header and
then reads the download table:

    lui   $s1, 0xa001
    lw    $s1, 0x106c($s1)   # protocol image + 0x6c = DspCodeBaseAddr
    ...
    lhu   $s2, ($s1)         # download count
    addiu $s1, $s1, 4        # -> t_dsp_portable_desc[], stride 0x30

Without this image the count reads 0, every DSP object is constructed with
an empty code table (`0x80085394`, fields +0/+4), and no overlay can ever be
assigned — which is the real reason a modem ASSIGN produces no host writes,
independent of how many DSPs the card init enumerates.

The layout reproduces `pri_telindus_load` (kernel/s_pri.c) and
`dsp_read_file` (divactrl/load/common/dsp_file.c) from the shipping Linux
driver:

    +0x0000  dword  download_count
    +0x0004  t_dsp_portable_desc[DSP_MAX_DOWNLOAD_COUNT]   (128 * 0x30)
    +0x1804  section data, dword-aligned, in dsp_read_file order

Each descriptor's seven pointer fields hold the card address of that
section, or 0 when the section is empty (`dsp_card_load_portable`).

Which downloads are staged is decided by the combifile itself: its
directory maps a `card_type_number` (a CARDTYPE_* value, e.g. 23 for
CARDTYPE_DIVASRV_P_30M_PCI) to a file-set number, and each download carries
a usage-mask bit for that file set.
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from eicon_dsp_extract import FILE_HEADER, FormatError, parse_combifile

# kernel/dsp_defs.h
DSP_MAX_DOWNLOAD_COUNT = 128
PORTABLE_DESC = struct.Struct("<10H7I")  # t_dsp_portable_desc, 0x30 bytes
assert PORTABLE_DESC.size == 0x30

# kernel/mi_pc.h
OFFS_DSP_CODE_BASE_ADDR = 0x6C
OFFS_PROTOCOL_END_ADDR = 0x7C

# kernel/cardtype.h: CARDTYPE_DIVASRV_P_30M_PCI, the card te_dmlt.pm targets.
CARDTYPE_DIVASRV_P_30M_PCI = 23


def _align4(value: int) -> int:
    return (value + 3) & ~3


@dataclass
class StagedDownload:
    download_id: int
    description: str
    address: int
    size: int


@dataclass
class DspCodeImage:
    base_addr: int
    data: bytes
    card_type: int
    file_set: int
    downloads: list[StagedDownload] = field(default_factory=list)

    @property
    def end_addr(self) -> int:
        return self.base_addr + len(self.data)


def required_downloads(combi: dict, card_type: int) -> tuple[list[dict], int]:
    """Select the downloads the combifile marks as required for a card type."""
    file_set = None
    for entry in combi["directory"]:
        if entry["card_type"] == card_type:
            file_set = entry["file_set"]
            break
    if file_set is None:
        raise FormatError(f"card type {card_type} is not in the combifile directory")
    mask_offset, mask_bit = file_set // 8, 1 << (file_set & 7)
    if mask_offset >= combi["usage_mask_size"]:
        raise FormatError(
            f"card type {card_type} maps to invalid file set {file_set}"
        )
    selected = [
        download
        for download in combi["downloads"]
        if bytes.fromhex(download["usage_mask"])[mask_offset] & mask_bit
    ]
    return selected, file_set


def build_dsp_code_image(
    combifile: Path,
    card_type: int = CARDTYPE_DIVASRV_P_30M_PCI,
    base_addr: int = 0,
    max_download_count: int = DSP_MAX_DOWNLOAD_COUNT,
) -> DspCodeImage:
    """Lay out the count + descriptor table + section data for `card_type`.

    `base_addr` is the card address the image will be written to; the
    descriptor pointers are absolute card addresses, so it must match where
    the image is actually placed.
    """
    combi = parse_combifile(combifile)
    selected, file_set = required_downloads(combi, card_type)
    if len(selected) > max_download_count:
        raise FormatError(
            f"download table overflow: {len(selected)} required downloads "
            f"exceed the {max_download_count}-entry table"
        )

    table_bytes = 4 + max_download_count * PORTABLE_DESC.size
    payload = bytearray()
    payload_base = _align4(base_addr + table_bytes)
    descriptors: list[bytes] = []
    staged: list[StagedDownload] = []

    def append_section(data: bytes) -> int:
        """Stage one section; returns its card address (0 when empty).

        Mirrors dsp_card_load_portable + pri_download_buffer: empty sections
        anchor at NULL, and the download pointer is dword-aligned after each.
        """
        if not data:
            return 0
        address = payload_base + len(payload)
        payload.extend(data)
        while len(payload) % 4:
            payload.append(0)
        return address

    for download in selected:
        raw = download["raw"]
        fields = FILE_HEADER.unpack_from(raw, 0)[1:]
        (
            _version,
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
        ) = fields
        excess_header_size = header_size - FILE_HEADER.size

        # Sections in the order dsp_read_file streams them out of the file.
        pos = FILE_HEADER.size
        sections = []
        for length in (
            excess_header_size,
            description_size,
            memory_table_size,
            segment_table_size,
            symbol_table_size,
            dm_size,
            pm_size,
        ):
            sections.append(raw[pos : pos + length])
            pos += length
        if pos != len(raw):
            raise FormatError(
                f"download 0x{download_id:04x}: section sizes sum to 0x{pos:x}, "
                f"record is 0x{len(raw):x} bytes"
            )

        start = payload_base + len(payload)
        pointers = [append_section(section) for section in sections]
        descriptors.append(
            PORTABLE_DESC.pack(
                download_id,
                flags,
                processing_power,
                interface_channels,
                excess_header_size,
                memory_count,
                segment_count,
                symbol_count,
                dm_count,
                pm_count,
                *pointers,
            )
        )
        staged.append(
            StagedDownload(
                download_id=download_id,
                description=download["description"],
                address=start,
                size=payload_base + len(payload) - start,
            )
        )

    image = bytearray(struct.pack("<I", len(selected)))
    for descriptor in descriptors:
        image.extend(descriptor)
    image.extend(bytes(table_bytes - len(image)))
    image.extend(bytes(payload_base - (base_addr + len(image))))
    image.extend(payload)

    return DspCodeImage(
        base_addr=base_addr,
        data=bytes(image),
        card_type=card_type,
        file_set=file_set,
        downloads=staged,
    )


def protocol_end_addr(image: Path) -> int:
    """DspCodeBaseAddr the driver derives from the protocol image header."""
    header = image.read_bytes()[:0x80]
    end = struct.unpack_from("<I", header, OFFS_PROTOCOL_END_ADDR)[0]
    if end == 0:
        raise FormatError("protocol image declares no end address")
    return _align4(end)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("combifile", type=Path)
    parser.add_argument("--card-type", type=lambda s: int(s, 0),
                        default=CARDTYPE_DIVASRV_P_30M_PCI)
    parser.add_argument("--image", type=Path,
                        default=Path("docs/firmware/te_dmlt.pm"),
                        help="protocol image the base address is derived from")
    parser.add_argument("--base", type=lambda s: int(s, 0), default=None,
                        help="override DspCodeBaseAddr")
    parser.add_argument("-o", "--output", type=Path)
    args = parser.parse_args()

    base = args.base if args.base is not None else protocol_end_addr(args.image)
    dsp_image = build_dsp_code_image(args.combifile, args.card_type, base)
    print(f"card type {dsp_image.card_type} -> file set {dsp_image.file_set}: "
          f"{len(dsp_image.downloads)} downloads")
    print(f"DspCodeBaseAddr 0x{dsp_image.base_addr:08x}..0x{dsp_image.end_addr:08x} "
          f"({len(dsp_image.data)} bytes)")
    for entry in dsp_image.downloads:
        print(f"  id=0x{entry.download_id:04x} @0x{entry.address:08x} "
              f"{entry.size:7d}  {entry.description}")
    if args.output:
        args.output.write_bytes(dsp_image.data)
        print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
