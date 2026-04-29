#!/usr/bin/env python3
import argparse
import csv
import re
import sys
from pathlib import Path
from typing import Dict, List, Set, Tuple

# Import the merge logic if possible, or replicate it
# Since I can't easily import from docs/tools due to pathing/execution environment, 
# I'll implement a simplified version of the logic found in regmap_merge_csv.py

def normalize_int_text(value: str) -> str:
    s = (value or "").strip()
    if not s: return ""
    try:
        return f"0x{int(s, 0):04X}"
    except ValueError:
        return ""

def load_and_merge_csvs(common_dir: Path, override_dir: Path) -> Dict[str, str]:
    """Loads registers.csv and merges with registers_override.csv. Returns map of address -> name."""
    reg_map = {}
    
    def process_file(path: Path):
        if not path.exists(): return
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                op = (row.get("op", "") or "upsert").strip().lower()
                
                # Check for range
                addr_start_str = row.get("address", "").strip()
                addr_end_str = row.get("address_end", "").strip()
                
                if not addr_start_str: continue
                
                try:
                    start = int(addr_start_str, 0)
                    end = int(addr_end_str, 0) if addr_end_str else start
                except ValueError:
                    continue
                
                for a in range(start, end + 1):
                    addr_hex = f"0x{a:04X}"
                    if op == "delete":
                        if addr_hex in reg_map:
                            del reg_map[addr_hex]
                    else:
                        reg_map[addr_hex] = row.get("name", "UNNAMED")

    # Load common
    process_file(common_dir / "registers.csv")
    # Load overrides
    process_file(override_dir / "registers_override.csv")
    
    return reg_map

def parse_regremap_c(c_file: Path) -> Set[str]:
    """Extracts hex addresses from 'case 0x...:' lines in regremap.c."""
    addresses = set()
    if not c_file.exists():
        return addresses
    
    # Regex to find 'case 0xXXX:' or 'case XXX:'
    # Handles 0x prefix and decimal.
    case_re = re.compile(r"^\s*case\s+(0x[0-9a-fA-F]+|[0-9]+)\s*:")
    
    with open(c_file, "r") as f:
        for line in f:
            match = case_re.search(line)
            if match:
                addr_str = match.group(1)
                addr = normalize_int_text(addr_str)
                if addr:
                    addresses.add(addr)
    return addresses

def validate(board_name: str, common_dir: Path, override_dir: Path, c_file: Path):
    csv_regs = load_and_merge_csvs(common_dir, override_dir)
    c_regs = parse_regremap_c(c_file)
    
    # Filter out registers named "reserved" from csv_regs for validation purposes
    active_csv_regs = {addr: name for addr, name in csv_regs.items() if name.lower() != "reserved"}
    csv_addrs = set(active_csv_regs.keys())
    
    output = []
    output.append(f"--- Validating {board_name} ---")
    
    # Filter out reserved memory window (often at 0xFFE0+) if needed, 
    # but usually regremap.c only handles lower addresses.
    
    only_in_csv = csv_addrs - c_regs
    only_in_c = c_regs - csv_addrs
    
    # Some common addresses might not be implemented in C if they are just placeholders or for other boards
    # but board-specific overrides definitely should be in C.
    
    if only_in_c:
        output.append(f"[ERROR] Found addresses in {c_file.name} that are NOT in CSV documentation (or marked as reserved):")
        for addr in sorted(only_in_c):
            output.append(f"  - {addr}")
    else:
        output.append("[OK] All addresses in C are documented in CSV.")
        
    if only_in_csv:
        # Note: Not necessarily an error if some documented regs aren't handled in firmware yet,
        # but good to know.
        output.append(f"[INFO] Found addresses in CSV documentation that are NOT implemented in {c_file.name}:")
        for addr in sorted(only_in_csv):
            output.append(f"  - {addr} ({active_csv_regs[addr]})")
    else:
        output.append("[OK] All documented CSV registers are implemented in C.")
    
    output.append("")
    
    output_str = "\n".join(output)
    print(output_str)
    return output_str

def main():
    parser = argparse.ArgumentParser(description="Validate regremap.c against CSV definitions.")
    parser.add_argument("--board", help="Specific board to validate (e.g. limesdr-xtrx)")
    parser.add_argument("--output", "-o", help="Output file to save the validation results")
    args = parser.parse_args()
    
    root = Path(__file__).parent.parent
    common_dir = root / "docs" / "docs" / "common_host_regs"
    
    boards = {
        "limesdr-xtrx": {
            "override": root / "docs" / "docs" / "limesdr-xtrx" / "reg_remap",
            "c_file": root / "firmware" / "bsp" / "LimeSDR_XTRX" / "regremap.c"
        },
        "limesdr-mini-v1": {
            "override": root / "docs" / "docs" / "limesdr-mini-v1" / "reg_remap",
            "c_file": root / "firmware" / "bsp" / "LimeSDR_Mini_V1" / "regremap.c"
        },
        "limesdr-mini-v2": {
            "override": root / "docs" / "docs" / "limesdr-mini-v2" / "reg_remap",
            "c_file": root / "firmware" / "bsp" / "LimeSDR_Mini_V2" / "regremap.c"
        },
        "hipersdr-44xx": {
            "override": root / "docs" / "docs" / "hipersdr-44xx" / "reg_remap",
            "c_file": root / "firmware" / "bsp" / "HiperSDR_44xx" / "hiper_regremap.c"
        },
        "ssdr_rev2": {
            "override": root / "docs" / "docs" / "ssdr_rev2" / "reg_remap",
            "c_file": root / "firmware" / "bsp" / "SSDR" / "regremap.c"
        }
    }
    
    results = []
    if args.board:
        if args.board in boards:
            b = boards[args.board]
            results.append(validate(args.board, common_dir, b["override"], b["c_file"]))
        else:
            print(f"Unknown board: {args.board}")
            sys.exit(1)
    else:
        for name, b in boards.items():
            results.append(validate(name, common_dir, b["override"], b["c_file"]))
            
    if args.output:
        with open(args.output, "w") as f:
            f.write("\n".join(results))
        print(f"Results saved to {args.output}")

if __name__ == "__main__":
    main()
