#!/usr/bin/env python3
import os
import sys
import re
import argparse
import subprocess

#
# This tool analyzes firmware build artifacts (ELF/MAP) to determine optimal memory region sizes.
# It identifies used space in each region and suggests "optimal" values.
#

BIOS_ROM_REQ = 0x4400
BIOS_SRAM_REQ = 0x800

VAR_MAPPING = {
    'rom': ['integrated_rom_size'],
    'sram': ['integrated_sram_size', 'integrated_sram_ram_size'],
    'main_ram': ['integrated_main_ram_size'],
}

BRAM_GRANULARITY = {
    'LIMESDR_MINI_V1': 2048, # ECP5
    'LIMESDR_MINI_V2': 2048, # ECP5
    'HIPERSDR_44XX': 4096,   # Xilinx 7-series
    'LIMESDR_XTRX': 4096,    # Xilinx 7-series
    'SSDR': 4096,            # Xilinx 7-series
}

class MemoryRegion:
    def __init__(self, name, origin, length):
        self.name = name
        self.origin = origin
        self.length = length
        self.used = 0
        self.sections = []

    def add_section(self, name, addr, size, kind='VMA'):
        if size == 0: return False
        end = addr + size
        if self.origin <= addr < self.origin + self.length:
            self.used = max(self.used, end - self.origin)
            self.sections.append({'name': name, 'addr': addr, 'size': size, 'kind': kind})
            return True
        return False

    def __str__(self):
        used_kb = self.used / 1024
        free_kb = (self.length - self.used) / 1024
        return f"{self.name:15} | 0x{self.origin:08x} | 0x{self.length:08x} | 0x{self.used:08x} | {used_kb:8.1f}K | {free_kb:8.1f}K"

def get_elf_sections(elf_path):
    """Analyze ELF using readelf to get loadable sections (ALLOC flag) and their LMAs."""
    sections = []
    try:
        # Get section headers
        res = subprocess.check_output(['readelf', '-S', '-W', elf_path], universal_newlines=True, stderr=subprocess.DEVNULL)
        # [Nr] Name Type Addr Off Size ES Flg ...
        row_re = re.compile(r'\[\s*\d+\]\s+(\.[a-zA-Z0-9_.]+)\s+\S+\s+([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+[0-9a-fA-F]+\s+(\S+)')
        for line in res.splitlines():
            m = row_re.search(line)
            if m:
                name, addr_hex, size_hex, flags = m.groups()
                if 'A' in flags: # ALLOC flag
                    sections.append({'name': name, 'vma': int(addr_hex, 16), 'size': int(size_hex, 16), 'lma': int(addr_hex, 16)})
        
        # Get LMAs from program headers
        res = subprocess.check_output(['readelf', '-l', '-W', elf_path], universal_newlines=True, stderr=subprocess.DEVNULL)
        load_re = re.compile(r'LOAD\s+\S+\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)')
        segments = []
        for line in res.splitlines():
            m = load_re.search(line)
            if m:
                segments.append({'vma': int(m.group(1), 16), 'lma': int(m.group(2), 16), 'filesiz': int(m.group(3), 16)})
        
        # Assign LMAs to sections
        for s in sections:
            for seg in segments:
                if seg['vma'] <= s['vma'] < seg['vma'] + seg['filesiz']:
                    s['lma'] = seg['lma'] + (s['vma'] - seg['vma'])
                    break
    except:
        return None
    return sections

def normalize_board_name(name):
    """Normalize board name to UPPER_CASE_WITH_UNDERSCORES."""
    if not name: return ""
    # Remove quotes, strip, uppercase, replace - with _
    return name.strip().replace('"', '').replace('-', '_').upper()

def get_target_file_vars(board_name):
    """Attempt to parse the corresponding target file for current definitions."""
    if not board_name or board_name == "Unknown":
        return {}
    
    board_lower = board_name.lower().replace('-', '_')
    target_dir = os.path.join('boards', 'targets')
    
    # Potential candidates for target file
    candidates = []
    
    # 1. Search in boards/targets/ with prefix matching (longest first)
    if os.path.exists(target_dir):
        # Exact match
        candidates.append(os.path.join(target_dir, board_lower + '.py'))
        # Prefixes
        parts = board_lower.split('_')
        for i in range(len(parts) - 1, 0, -1):
            candidates.append(os.path.join(target_dir, '_'.join(parts[:i]) + '.py'))

    # 2. Search in root with prefix matching
    candidates.append(board_lower + '.py')
    parts = board_lower.split('_')
    for i in range(len(parts) - 1, 0, -1):
        candidates.append('_'.join(parts[:i]) + '.py')

    target_file = None
    for c in candidates:
        if os.path.exists(c):
            target_file = c
            break

    if not target_file:
        return {}

    vars = {}
    try:
        with open(target_file, 'r') as f:
            content = f.read()
            # Match integrated_rom_size = 0x8800 or 1234
            for var_list in VAR_MAPPING.values():
                for var in var_list:
                    # Find all matches (can be multiple for different build modes)
                    matches = re.findall(fr'{var}\s*=\s*(0x[0-9a-fA-F]+|\d+)', content)
                    if matches:
                        vals = []
                        for val_str in matches:
                            if val_str.startswith('0x'):
                                vals.append(int(val_str, 16))
                            else:
                                vals.append(int(val_str))
                        vars[var] = vals
    except:
        pass
    return vars

def detect_board_name(path, elf_file):
    """Attempt to detect the board name from various sources."""
    # 1. Try env.mak in same dir or parent (firmware root)
    search_dirs = [os.path.dirname(path), os.path.join(os.path.dirname(path), '..')]
    for d in search_dirs:
        if not d: continue
        env_mak = os.path.join(d, 'env.mak')
        if os.path.exists(env_mak):
            try:
                with open(env_mak, 'r') as f:
                    for line in f:
                        if line.startswith('TARGET='):
                            return normalize_board_name(line.split('=')[1])
            except: pass

    # 2. Try to infer from path (build/<board>/...)
    path_parts = os.path.abspath(path).split(os.sep)
    try:
        idx = path_parts.index('build')
        if idx + 1 < len(path_parts):
            return normalize_board_name(path_parts[idx + 1])
    except ValueError:
        pass

    # 3. Try LiteX ident string in ELF
    if elf_file and os.path.exists(elf_file):
        try:
            res = subprocess.check_output(['strings', elf_file], universal_newlines=True, stderr=subprocess.DEVNULL)
            for line in res.splitlines():
                if 'LiteX SoC on' in line:
                    # Extract board name: LiteX SoC on <Board> <Date>
                    m = re.search(r'LiteX SoC on\s+(.*?)\s+\d{4}-\d{2}-\d{2}', line)
                    if m:
                        return normalize_board_name(m.group(1))
                    # Fallback if date is missing
                    m = re.search(r'LiteX SoC on\s+(.*)', line)
                    if m:
                        return normalize_board_name(m.group(1))
        except:
            pass

    # 4. Try to infer from filename if it's not generic
    filename = os.path.basename(path)
    generics = ['firmware', 'bios', 'soc', 'output', 'build', 'top', 'main']
    name_no_ext = os.path.splitext(filename)[0]
    if name_no_ext and not any(g in name_no_ext.lower() for g in generics):
        return normalize_board_name(name_no_ext)

    return "Unknown"

def analyze_artifacts(base_path):
    """Try to find MAP and/or ELF for the given path and analyze them."""
    map_file = None
    elf_file = None
    
    if os.path.isdir(base_path):
        # Search for .map and .elf in directory
        for f in os.listdir(base_path):
            if f.endswith('.map'): map_file = os.path.join(base_path, f)
            if f.endswith('.elf'): elf_file = os.path.join(base_path, f)
    elif base_path.endswith('.map'):
        map_file = base_path
        elf_candidate = base_path[:-4] + '.elf'
        if os.path.exists(elf_candidate): elf_file = elf_candidate
    elif base_path.endswith('.elf'):
        elf_file = base_path
        map_candidate = base_path + '.map'
        if os.path.exists(map_candidate): map_file = map_candidate
    else:
        # Just check if it exists
        if os.path.exists(base_path):
            # Try to determine type
            if '.map' in base_path: map_file = base_path
            if '.elf' in base_path: elf_file = base_path

    if not map_file:
        print(f"Error: Could not find MAP file for {base_path}")
        return

    # Parse MAP for Memory Configuration (Regions)
    with open(map_file, 'r') as f:
        content = f.read()

    mem_cfg_match = re.search(r'Memory Configuration\s+Name\s+Origin\s+Length(.*?)\n\n', content, re.DOTALL)
    if not mem_cfg_match:
        print(f"Error: Could not find 'Memory Configuration' in {map_file}")
        return

    regions = {}
    region_re = re.compile(r'^\s*(\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)')
    for line in mem_cfg_match.group(1).strip().split('\n'):
        m = region_re.search(line)
        if m:
            name, origin, length = m.groups()
            if name == '*default*': continue
            regions[name] = MemoryRegion(name, int(origin, 16), int(length, 16))

    # Get sections either from ELF (preferred for flags) or MAP
    sections = []
    if elf_file:
        sections = get_elf_sections(elf_file)
    
    if not sections:
        # Fallback to MAP sections
        section_re = re.compile(r'^(\.[a-zA-Z0-9_.]+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)(?:\s+load address\s+(0x[0-9a-fA-F]+))?', re.MULTILINE)
        for m in section_re.finditer(content):
            name, vma, size, lma = m.groups()
            # Filter out non-loadable sections
            if any(name.startswith(p) for p in ['.debug', '.comment', '.riscv.attributes', '.note', '.symtab', '.strtab']):
                continue
            sections.append({
                'name': name,
                'vma': int(vma, 16),
                'size': int(size, 16),
                'lma': int(lma, 16) if lma else int(vma, 16)
            })

    # Map sections to regions
    for s in sections:
        for r in regions.values():
            r.add_section(s['name'], s['vma'], s['size'], 'VMA')
            if s['lma'] != s['vma']:
                r.add_section(s['name'], s['lma'], s['size'], 'LMA')

    return map_file, elf_file, regions

def main():
    parser = argparse.ArgumentParser(description='Analyze firmware memory usage.')
    parser.add_argument('paths', nargs='*', default=['firmware'], help='Path to .map/.elf files or build directories (default: firmware)')
    parser.add_argument('--round', type=int, default=1024, help='Rounding for optimal values (default 1024)')
    parser.add_argument('--stack', type=int, default=1024, help='Stack safety margin for SRAM (default 1024)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Show detailed section-level breakdown')
    args = parser.parse_args()

    for path in args.paths:
        result = analyze_artifacts(path)
        if not result: continue
        map_file, elf_file, regions = result

        board = detect_board_name(path, elf_file)
        target_vars = get_target_file_vars(board)
        
        # BRAM Granularity awareness (Proposal 3)
        round_inc = args.round
        is_hardware = False
        # Try exact match or prefix match for granularity
        for b_key, b_gran in BRAM_GRANULARITY.items():
            if board == b_key or board.startswith(b_key + '_'):
                round_inc = b_gran
                is_hardware = True
                break
        
        print(f"\nAnalysis for: {map_file}")
        print(f"Board       : {board} ({'Target defs loaded' if target_vars else 'No target file found'})")
        print(f"Granularity : {round_inc} bytes ({'Hardware BRAM' if is_hardware else 'Default'})")
        
        print("-" * 84)
        print(f" {'Region':15} | {'Origin':10} | {'Length':10} | {'Used (Hex)':10} | {'Used':9} | {'Free':9}")
        print("-" * 17 + "+" + "-" * 12 + "+" + "-" * 12 + "+" + "-" * 12 + "+" + "-" * 11 + "+" + "-" * 11)
        for name in sorted(regions.keys(), key=lambda x: regions[x].origin):
            print(f" {regions[name]}")
            if args.verbose:
                # Sort sections by size descending
                for s in sorted(regions[name].sections, key=lambda x: x['size'], reverse=True):
                    print(f"   - {s['name']:14} | 0x{s['addr']:08x} | {s['size']:10} bytes ({s['kind']})")
            
        def print_recs(bios_enabled):
            title = "Recommended for BIOS-enabled SoC" if bios_enabled else "Recommended for BIOS-disabled SoC"
            print(f"\n{title}:")
            
            header = f" {'Region':15} | {'LiteX Variable':25} | {'Optimal':10} | {'Current':10} | {'Delta'}"
            print(header)
            print("-" * 17 + "+" + "-" * 27 + "+" + "-" * 12 + "+" + "-" * 12 + "+" + "-" * 15)

            # Identify all relevant regions
            region_names = set(regions.keys())
            if bios_enabled:
                region_names.add('rom')
                region_names.add('sram')
            
            for name in sorted(list(region_names), key=lambda x: regions[x].origin if x in regions else 0):
                r = regions.get(name)
                used = r.used if r else 0
                current_len = r.length if r else 0
                # Base used calculation
                base_used = used
                bios_details = []
                if bios_enabled:
                    if name.lower() == 'rom':
                        base_used += BIOS_ROM_REQ
                        bios_details.append(f"BIOS: 0x{BIOS_ROM_REQ:x}")
                    elif name.lower() == 'sram':
                        base_used += BIOS_SRAM_REQ
                        bios_details.append(f"BIOS: 0x{BIOS_SRAM_REQ:x}")
                
                # Stack margin for SRAM (standard LiteX stack location)
                if 'sram' in name.lower():
                    # Always add stack margin to SRAM if it exists, as it's the primary stack location
                    base_used += args.stack
                    bios_details.append(f"Stack: 0x{args.stack:x}")
                
                # Skip if not used and not a required BIOS/Stack region
                if base_used == 0: continue
                
                optimal = ((base_used + round_inc - 1) // round_inc) * round_inc
                
                # LiteX variable mapping (Proposal 1)
                litex_vars = VAR_MAPPING.get(name.lower(), [])
                litex_var = ""
                defined_size = current_len
                
                if litex_vars:
                    # Find which variable is actually in target_vars
                    for v in litex_vars:
                        if v in target_vars:
                            litex_var = v
                            # target_vars[v] is a list of all found assignments
                            possible_vals = target_vars[v]
                            # Try to find the one that matches current_len (most accurate)
                            if current_len in possible_vals:
                                defined_size = current_len
                            else:
                                # Fallback to the last one (most likely final assignment)
                                defined_size = possible_vals[-1]
                            break
                    if not litex_var:
                        # Fallback to first one in list for display
                        litex_var = litex_vars[0]

                # Delta calculation (Proposal 2)
                delta = defined_size - optimal
                if defined_size > 0:
                    if delta > 0:
                        delta_str = f"-0x{delta:x} (-{delta/1024:.1f}K)"
                    elif delta < 0:
                        delta_str = f"+0x{-delta:x} (+{-delta/1024:.1f}K)"
                    else:
                        delta_str = "Match"
                else:
                    delta_str = "-"

                # Details for indented line
                details_list = []
                if used > 0: details_list.append(f"App: 0x{used:x}")
                details_list.extend(bios_details)
                if defined_size != current_len and current_len > 0 and defined_size > 0:
                    details_list.append(f"MAP mismatch: 0x{current_len:x}")
                
                print(f" {name:15} | {litex_var:25} | 0x{optimal:08x} | 0x{defined_size:08x} | {delta_str}")
                if details_list:
                    print(f" {' ':15} |   ({', '.join(details_list)})")

        print_recs(bios_enabled=True)
        print_recs(bios_enabled=False)
        print("\n" + "="*78 + "\n")

if __name__ == "__main__":
    main()
