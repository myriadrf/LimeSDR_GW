#!/usr/bin/env python3
import csv
import subprocess
import os
import sys
from pathlib import Path

def run_make():
    print("Running make regmap-all in docs...")
    # Using make -i to ignore errors in RST generation as we only need CSVs
    subprocess.run(["make", "-i", "regmap-all"], cwd="docs", check=False)

def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, 'r', encoding='utf-8') as f:
        return list(csv.DictReader(f))

def get_boards():
    # Detect boards from /tmp/regmap files
    boards = []
    if not os.path.exists('/tmp/regmap'):
        return []
    for f in os.listdir('/tmp/regmap'):
        if f.endswith('_regmap_modules.csv'):
            board = f.replace('_regmap_modules.csv', '')
            boards.append(board)
    return sorted(boards)

def is_reserved(text):
    if not text:
        return True
    return text.lower().startswith('reserved')

def main():
    root_dir = Path(__file__).resolve().parent.parent
    os.chdir(root_dir)
    
    run_make()
    boards = get_boards()
    if not boards:
        print("Error: No board data found in /tmp/regmap. Make sure 'make regmap-all' works.")
        sys.exit(1)
        
    print(f"Detected boards: {', '.join(boards)}")

    # Load common data
    common_dir = root_dir / 'docs' / 'docs' / 'common_host_regs'
    common_regs_list = read_csv(common_dir / 'registers.csv')
    common_modules_list = read_csv(common_dir / 'modules.csv')
    
    # Map common regs by address
    common_regs = {}
    for r in common_regs_list:
        if not r['address']: continue
        addr_start = int(r['address'], 16)
        addr_end = int(r['address_end'], 16) if r.get('address_end') else addr_start
        for addr in range(addr_start, addr_end + 1):
            common_regs[addr] = r

    # Load board data
    board_data = {}
    all_modules = {} # module_name -> (start, end)
    
    # Initialize with common modules
    for m in common_modules_list:
        all_modules[m['module']] = (int(m['address_start'], 16), int(m['address_end'], 16))

    for board in boards:
        regs_list = read_csv(f'/tmp/regmap/{board}_regmap_registers.csv')
        modules_list = read_csv(f'/tmp/regmap/{board}_regmap_modules.csv')
        
        board_regs = {}
        for r in regs_list:
            if not r['address']: continue
            addr_start = int(r['address'], 16)
            addr_end = int(r['address_end'], 16) if r.get('address_end') else addr_start
            for addr in range(addr_start, addr_end + 1):
                board_regs[addr] = r
        board_data[board] = board_regs
        
        for m in modules_list:
            name = m['module']
            start = int(m['address_start'], 16)
            end = int(m['address_end'], 16)
            if name in all_modules:
                cur_start, cur_end = all_modules[name]
                all_modules[name] = (min(cur_start, start), max(cur_end, end))
            else:
                all_modules[name] = (start, end)

    # Sort modules by address
    sorted_modules = sorted(all_modules.items(), key=lambda x: x[1][0])

    # Prepare output columns
    header = ['Module', 'Address', 'Master', 'Usage', 'Common Name', 'Common Description']
    for board in boards:
        header.append(f'{board} Name')
        header.append(f'{board} Description')

    output_rows = []
    
    for mod_name, (mod_start, mod_end) in sorted_modules:
        for addr in range(mod_start, mod_end + 1):
            addr_hex = f"0x{addr:04X}"
            row = {
                'Module': mod_name,
                'Address': addr_hex
            }
            
            common = common_regs.get(addr, {})
            c_name = common.get('name', '')
            c_desc = common.get('description', '')
            row['Common Name'] = c_name
            row['Common Description'] = c_desc
            
            used_boards = []
            if not is_reserved(c_name) or not is_reserved(c_desc):
                used_boards.append('Common')
            
            for board in boards:
                br = board_data[board].get(addr, {})
                name = br.get('name', '')
                desc = br.get('description', '')
                row[f'{board} Name'] = name
                row[f'{board} Description'] = desc
                
                if not is_reserved(name) or not is_reserved(desc):
                    used_boards.append(board)
            
            # Master logic: UNUSED if all name/description columns are either empty or start with reserved
            all_name_desc = [c_name, c_desc]
            for board in boards:
                all_name_desc.append(row[f'{board} Name'])
                all_name_desc.append(row[f'{board} Description'])
            
            strictly_unused = True
            for val in all_name_desc:
                if not is_reserved(val):
                    strictly_unused = False
                    break
            
            row['Master'] = 'UNUSED' if strictly_unused else 'USED'
            row['Usage'] = ', '.join(used_boards) if used_boards else ''
            
            output_rows.append(row)

    output_file = root_dir / 'master_registers.csv'
    with open(output_file, 'w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        writer.writerows(output_rows)
    print(f"Generated {output_file}")

if __name__ == "__main__":
    main()
