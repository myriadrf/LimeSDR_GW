#!/usr/bin/env python3
"""Generate an RST register-map document from CSV inputs.

CSV inputs:
1) modules.csv columns:
   module,address_start,address_end,typical_use
2) registers.csv columns:
   module,address,address_end,default,name,description
   - address_end is optional; if empty, address_end == address
3) bitfields.csv columns:
   address,title,access,default,field,msb,lsb,values,description
   - one row per bitfield
   - title/access/default can be repeated or left empty after first row per register
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List


@dataclass
class ModuleRow:
    module: str
    address_start: int
    address_end: int
    typical_use: str


@dataclass
class RegisterRow:
    module: str
    address: int
    address_end: int
    default: str
    name: str
    description: str


@dataclass
class BitfieldRow:
    address: int
    title: str
    access: str
    default: str
    field: str
    msb: int
    lsb: int
    values: str
    description: str


def is_reserved_name(name: str) -> bool:
    return name.strip().lower() == "reserved"


def parse_int(text: str) -> int:
    s = (text or "").strip()
    if not s:
        raise ValueError("empty integer field")
    return int(s, 0)


def normalize_default(text: str) -> str:
    s = (text or "").strip()
    if s == "-":
        return ""
    return s


def as_hex(addr: int) -> str:
    return f"0x{addr:04X}"


def read_modules(path: Path) -> List[ModuleRow]:
    rows: List[ModuleRow] = []
    with path.open(newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            rows.append(
                ModuleRow(
                    module=raw["module"].strip(),
                    address_start=parse_int(raw["address_start"]),
                    address_end=parse_int(raw["address_end"]),
                    typical_use=raw.get("typical_use", "").strip(),
                )
            )
    return rows


def read_registers(path: Path) -> List[RegisterRow]:
    rows: List[RegisterRow] = []
    with path.open(newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            a = parse_int(raw["address"])
            b = parse_int(raw["address_end"]) if raw.get("address_end", "").strip() else a
            rows.append(
                RegisterRow(
                    module=raw["module"].strip(),
                    address=a,
                    address_end=b,
                    default=normalize_default(raw.get("default", "")),
                    name=raw.get("name", "").strip() or "Reserved",
                    description=raw.get("description", "").strip() or "Reserved.",
                )
            )
    return rows


def read_bitfields(path: Path) -> List[BitfieldRow]:
    rows: List[BitfieldRow] = []
    with path.open(newline="", encoding="utf-8") as f:
        for raw in csv.DictReader(f):
            rows.append(
                BitfieldRow(
                    address=parse_int(raw["address"]),
                    title=raw.get("title", "").strip(),
                    access=raw.get("access", "").strip(),
                    default=normalize_default(raw.get("default", "")),
                    field=raw["field"].strip(),
                    msb=parse_int(raw["msb"]),
                    lsb=parse_int(raw["lsb"]),
                    values=raw.get("values", "").strip(),
                    description=raw.get("description", "").strip(),
                )
            )
    return rows


def underline(text: str, ch: str) -> str:
    return ch * len(text)


def bit_label(msb: int, lsb: int) -> str:
    if msb == lsb:
        return f"[{msb}]"
    return f"[{msb}:{lsb}]"


def emit_cell(value: str) -> str:
    v = (value or "").strip()
    # Use a non-breaking space in empty cells to prevent Sphinx/docutils from
    # rendering list artifacts for truly empty list-table entries.
    return f"     - {v}" if v else "     - \u00A0"


def make_wavedrom(fields: List[BitfieldRow], reg_width: int = 16) -> str:
    parts: List[str] = []

    def append_chunks(msb: int, lsb: int, name: str) -> None:
        """Append field chunks split on 8-bit boundaries for clearer 2x8 rendering."""
        total_width = msb - lsb + 1
        cur_lsb = lsb
        while cur_lsb <= msb:
            chunk_msb = min(msb, ((cur_lsb // 8) + 1) * 8 - 1)
            width = chunk_msb - cur_lsb + 1
            if total_width == 1:
                label = name
            else:
                label = f"{name} [{chunk_msb}:{cur_lsb}]"
            parts.append(f'{{"bits": {width}, "name": "{label}"}}')
            cur_lsb = chunk_msb + 1

    # WaveDrom register fields are emitted from LSB to MSB.
    fields_sorted = sorted(fields, key=lambda x: (x.lsb, x.msb))
    next_lsb = 0
    for f in fields_sorted:
        if f.msb < f.lsb:
            raise ValueError(f"invalid field range for {f.field}: msb < lsb")
        if f.lsb < next_lsb:
            raise ValueError(f"overlap/out-of-order near field {f.field}")
        if f.lsb > next_lsb:
            append_chunks(f.lsb - 1, next_lsb, "Reserved")
        append_chunks(f.msb, f.lsb, f.field)
        next_lsb = f.msb + 1
    if next_lsb < reg_width:
        append_chunks(reg_width - 1, next_lsb, "Reserved")
    return (
        ".. wavedrom::\n\n"
        "   { \"reg\": [\n"
        "     " + ",\n     ".join(parts) + "\n"
        f"   ], \"config\": {{ \"bits\": {reg_width}, \"lanes\": 2, \"hspace\": 1150 }} }}\n"
    )


def render(
    title: str,
    modules: List[ModuleRow],
    registers: List[RegisterRow],
    bitfields: List[BitfieldRow],
    label_prefix: str,
) -> str:
    mod_regs: Dict[str, List[RegisterRow]] = {m.module: [] for m in modules}
    for r in registers:
        mod_regs.setdefault(r.module, []).append(r)
    for key in mod_regs:
        mod_regs[key].sort(key=lambda x: (x.address, x.address_end))

    bf_by_addr: Dict[int, List[BitfieldRow]] = {}
    for b in bitfields:
        bf_by_addr.setdefault(b.address, []).append(b)
    for addr in bf_by_addr:
        bf_by_addr[addr].sort(key=lambda x: (-x.msb, -x.lsb))

    # Prefer register-table names for per-address bitfield section headings.
    reg_name_by_addr: Dict[int, str] = {}
    for r in registers:
        if r.address == r.address_end:
            reg_name_by_addr[r.address] = r.name

    # Ensure every single-address register has bitfield coverage.
    # - Reserved registers get an auto-generated full-width reserved field if missing.
    # - Non-reserved registers without bitfields are an error.
    missing_non_reserved: List[RegisterRow] = []
    for r in registers:
        if r.address != r.address_end:
            continue
        if r.address in bf_by_addr:
            continue
        if is_reserved_name(r.name):
            bf_by_addr[r.address] = [
                BitfieldRow(
                    address=r.address,
                    title="reserved",
                    access="R/W",
                    default=r.default,
                    field="reserved",
                    msb=15,
                    lsb=0,
                    values="",
                    description=r.description or "Reserved.",
                )
            ]
        else:
            missing_non_reserved.append(r)
    if missing_non_reserved:
        entries = ", ".join(f"{as_hex(r.address)} ({r.name})" for r in missing_non_reserved)
        raise ValueError(
            "Missing bitfield rows for non-reserved registers: "
            + entries
            + ". Add rows to bitfields.csv."
        )

    out: List[str] = []
    out.append(title)
    out.append(underline(title, "="))
    out.append("")
    out.append("This page is generated from CSV sources.")
    out.append("")
    out.append("Quick Navigation")
    out.append("----------------")
    out.append("")
    out.append(".. list-table:: Module overview")
    out.append("   :header-rows: 1")
    out.append("   :widths: 20 16 64")
    out.append("")
    out.append("   * - Module")
    out.append("     - Address range")
    out.append("     - Typical use")
    for m in modules:
        anchor = f"{label_prefix}_regmap_{m.module.lower()}"
        out.append(f"   * - :ref:`{m.module} <{anchor}>`")
        out.append(f"     - ``{as_hex(m.address_start)}`` - ``{as_hex(m.address_end)}``")
        out.append(f"     - {m.typical_use}")
    out.append("")

    for m in modules:
        anchor = f"{label_prefix}_regmap_{m.module.lower()}"
        sec = f"{m.module} Registers (``{as_hex(m.address_start)}`` - ``{as_hex(m.address_end)}``)"
        out.append(f".. _{anchor}:")
        out.append("")
        out.append(sec)
        out.append(underline(sec, "-"))
        out.append("")
        out.append(f".. list-table:: {m.module} registers")
        out.append("   :header-rows: 1")
        out.append("   :widths: 10 12 20 58")
        out.append("")
        out.append("   * - Address")
        out.append("     - Default")
        out.append("     - Name")
        out.append("     - Description")
        for r in mod_regs.get(m.module, []):
            module_anchor = f"{label_prefix}_regmap_{r.module.lower()}"
            if r.address == r.address_end:
                if r.address in bf_by_addr:
                    anchor = f"{label_prefix}_reg_{r.address:04X}".lower()
                    addr = f":ref:`{as_hex(r.address)} <{anchor}>`"
                else:
                    addr = f":ref:`{as_hex(r.address)} <{module_anchor}>`"
            else:
                addr = (
                    f":ref:`{as_hex(r.address)} <{module_anchor}>` - "
                    f":ref:`{as_hex(r.address_end)} <{module_anchor}>`"
                )
            out.append(f"   * - {addr}")
            out.append(emit_cell(f"``{r.default}``" if r.default else ""))
            out.append(emit_cell(f"``{r.name}``" if not is_reserved_name(r.name) else "\\-"))
            out.append(emit_cell(r.description))
        out.append("")

    out.append("Register Bitfield Reference")
    out.append("-------------------")
    out.append("")
    out.append("Bit fields below are shown from MSB to LSB where applicable.")
    out.append("")

    for addr in sorted(bf_by_addr):
        rows = bf_by_addr[addr]
        first = rows[0]
        meta_title = next((r.title for r in rows if r.title), "")
        meta_access = next((r.access for r in rows if r.access), "")
        meta_default = next((r.default for r in rows if r.default), "")
        anchor = f"{label_prefix}_reg_{addr:04X}".lower()
        title_text = reg_name_by_addr.get(addr, "") or meta_title or first.field
        access = meta_access or "R/W"
        default = meta_default or "not specified"
        heading = f"``{as_hex(addr)}`` - {title_text}"
        out.append(f".. _{anchor}:")
        out.append("")
        out.append(heading)
        out.append(underline(heading, "^"))
        out.append("")
        out.append(f"Address: ``{as_hex(addr)}`` | Default: ``{default}`` | Access: {access}")
        out.append("")
        out.append(make_wavedrom(rows, reg_width=16))
        out.append(".. list-table::")
        out.append("   :header-rows: 1")
        out.append("   :widths: 12 22 26 40")
        out.append("")
        out.append("   * - Bit(s)")
        out.append("     - Field")
        out.append("     - Values")
        out.append("     - Description")
        for r in rows:
            out.append(f"   * - ``{bit_label(r.msb, r.lsb)}``")
            out.append(emit_cell(f"``{r.field}``" if not is_reserved_name(r.field) else "\\-"))
            out.append(emit_cell(r.values))
            out.append(emit_cell(r.description))
        out.append("")

    return "\n".join(out).rstrip() + "\n"


def main() -> None:
    p = argparse.ArgumentParser(description="Generate register-map RST from CSV files.")
    p.add_argument("--modules-csv", required=True, type=Path)
    p.add_argument("--registers-csv", required=True, type=Path)
    p.add_argument("--bitfields-csv", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--title", default="LimeSDR XTRX Register Map")
    p.add_argument("--label-prefix", default="xtrx")
    args = p.parse_args()

    text = render(
        title=args.title,
        modules=read_modules(args.modules_csv),
        registers=read_registers(args.registers_csv),
        bitfields=read_bitfields(args.bitfields_csv),
        label_prefix=args.label_prefix.strip(),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
