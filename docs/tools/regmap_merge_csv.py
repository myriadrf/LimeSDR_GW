#!/usr/bin/env python3
"""Merge common register-map CSVs with optional board-specific overrides.

Common files (required in --common-dir):
- modules.csv
- registers.csv
- bitfields.csv

Override files (optional in --override-dir):
- modules_override.csv
- registers_override.csv
- bitfields_override.csv

Override rows support an optional `op` column:
- upsert (default): replace existing row by key or append new row
- delete: remove row by key
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


MODULE_FIELDS = ["module", "address_start", "address_end", "typical_use"]
REGISTER_FIELDS = ["module", "address", "address_end", "default", "name", "description"]
BITFIELD_FIELDS = ["address", "title", "access", "default", "field", "msb", "lsb", "values", "description"]


def read_rows(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def normalize_int_text(value: str, fallback: str = "") -> str:
    s = (value or "").strip()
    if not s:
        s = fallback.strip()
    if not s:
        return ""
    return f"0x{int(s, 0):04X}"


def module_key(row: Dict[str, str]) -> Tuple[str]:
    return (row.get("module", "").strip().lower(),)


def register_key(row: Dict[str, str]) -> Tuple[str, str, str]:
    module = row.get("module", "").strip().lower()
    address = normalize_int_text(row.get("address", ""))
    address_end = normalize_int_text(row.get("address_end", ""), fallback=address)
    return (module, address, address_end)


def bitfield_key(row: Dict[str, str]) -> Tuple[str, int, int, str]:
    address = normalize_int_text(row.get("address", ""))
    msb = int((row.get("msb", "") or "0").strip(), 0)
    lsb = int((row.get("lsb", "") or "0").strip(), 0)
    field = (row.get("field", "") or "").strip().lower()
    return (address, msb, lsb, field)


def sanitize_row(row: Dict[str, str], fields: Sequence[str]) -> Dict[str, str]:
    return {k: (row.get(k, "") or "").strip() for k in fields}


def merge_rows(
    base_rows: List[Dict[str, str]],
    override_rows: List[Dict[str, str]],
    fields: Sequence[str],
    key_fn,
) -> List[Dict[str, str]]:
    result: List[Dict[str, str]] = [sanitize_row(r, fields) for r in base_rows]
    index: Dict[Tuple, int] = {key_fn(r): i for i, r in enumerate(result)}

    for raw in override_rows:
        op = (raw.get("op", "") or "upsert").strip().lower()
        row = sanitize_row(raw, fields)
        key = key_fn(row)
        if op == "delete":
            if key in index:
                idx = index.pop(key)
                result[idx] = None  # type: ignore[assignment]
            continue
        if op not in ("", "upsert"):
            raise ValueError(f"Unsupported op '{op}' for key {key}")
        if key in index:
            result[index[key]] = row
        else:
            index[key] = len(result)
            result.append(row)

    return [r for r in result if r is not None]


def write_rows(path: Path, fields: Sequence[str], rows: List[Dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(fields))
        w.writeheader()
        w.writerows(rows)


def maybe_read(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    return read_rows(path)


def main() -> None:
    p = argparse.ArgumentParser(description="Merge common and override register-map CSV files.")
    p.add_argument("--common-dir", required=True, type=Path)
    p.add_argument("--override-dir", required=True, type=Path)
    p.add_argument("--out-modules", required=True, type=Path)
    p.add_argument("--out-registers", required=True, type=Path)
    p.add_argument("--out-bitfields", required=True, type=Path)
    args = p.parse_args()

    common_modules = read_rows(args.common_dir / "modules.csv")
    common_registers = read_rows(args.common_dir / "registers.csv")
    common_bitfields = read_rows(args.common_dir / "bitfields.csv")

    ov_modules = maybe_read(args.override_dir / "modules_override.csv")
    ov_registers = maybe_read(args.override_dir / "registers_override.csv")
    ov_bitfields = maybe_read(args.override_dir / "bitfields_override.csv")

    merged_modules = merge_rows(common_modules, ov_modules, MODULE_FIELDS, module_key)
    merged_registers = merge_rows(common_registers, ov_registers, REGISTER_FIELDS, register_key)
    merged_bitfields = merge_rows(common_bitfields, ov_bitfields, BITFIELD_FIELDS, bitfield_key)

    write_rows(args.out_modules, MODULE_FIELDS, merged_modules)
    write_rows(args.out_registers, REGISTER_FIELDS, merged_registers)
    write_rows(args.out_bitfields, BITFIELD_FIELDS, merged_bitfields)


if __name__ == "__main__":
    main()
