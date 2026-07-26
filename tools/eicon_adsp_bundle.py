#!/usr/bin/env python3
"""Compose ordered Eicon ADSP extraction directories into one PM/DM image.

Later modules replace earlier words.  This models the firmware's staged task and
overlay loading; use --allow-conflicts explicitly when non-identical words
occupy the same flattened ADSP address.  Real hardware can additionally use
PMOVLAY/DMOVLAY banks, which the standalone emulator does not yet model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def read_words(path: Path) -> dict[int, int]:
    words: dict[int, int] = {}
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        fields = line.split()
        if not fields:
            continue
        if len(fields) != 2:
            raise ValueError(f"{path}:{line_number}: expected ADDRESS VALUE")
        address, value = (int(field, 16) for field in fields)
        if not 0 <= address < 0x10000:
            raise ValueError(f"{path}:{line_number}: address out of range")
        words[address] = value
    return words


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("modules", nargs="+", type=Path,
                        help="ordered extraction directories; later modules win")
    parser.add_argument("-o", "--output", type=Path, required=True,
                        help="output directory")
    parser.add_argument("--allow-conflicts", action="store_true",
                        help="allow later modules to replace differing words")
    args = parser.parse_args()

    images: dict[str, dict[int, tuple[int, str]]] = {"pm": {}, "dm": {}}
    conflicts: list[dict] = []
    module_names: list[str] = []

    for module in args.modules:
        metadata_path = module / "metadata.json"
        if not metadata_path.is_file():
            parser.error(f"not an extraction directory: {module}")
        metadata = json.loads(metadata_path.read_text())
        name = metadata.get("description", module.name)
        module_names.append(name)
        for domain in ("pm", "dm"):
            for address, value in read_words(module / f"{domain}.words").items():
                previous = images[domain].get(address)
                if previous is not None and previous[0] != value:
                    conflicts.append({
                        "domain": domain,
                        "address": address,
                        "before": previous[0],
                        "before_module": previous[1],
                        "after": value,
                        "after_module": name,
                    })
                images[domain][address] = (value, name)

    if conflicts and not args.allow_conflicts:
        first = conflicts[0]
        parser.error(
            f"{len(conflicts)} conflicting words; first is "
            f"{first['domain'].upper()} 0x{first['address']:04x} "
            f"({first['before_module']} -> {first['after_module']}); "
            "use --allow-conflicts for an intentional staged overlay"
        )

    args.output.mkdir(parents=True, exist_ok=True)
    for domain, width in (("pm", 3), ("dm", 2)):
        binary = bytearray(0x10000 * width)
        text: list[str] = []
        for address, (value, _) in sorted(images[domain].items()):
            binary[address * width:(address + 1) * width] = value.to_bytes(width, "little")
            text.append(f"{address:04x} {value:0{width * 2}x}\n")
        (args.output / f"{domain}.bin").write_bytes(binary)
        (args.output / f"{domain}.words").write_text("".join(text))

    manifest = {
        "modules": module_names,
        "loaded_pm_words": len(images["pm"]),
        "loaded_dm_words": len(images["dm"]),
        "conflict_count": len(conflicts),
        "conflicts": conflicts,
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        f"{args.output}: PM={manifest['loaded_pm_words']} "
        f"DM={manifest['loaded_dm_words']} conflicts={len(conflicts)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
