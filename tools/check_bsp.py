#!/usr/bin/env python3
import os
import re
import sys
import argparse
import csv
from pathlib import Path

def get_bsp_functions(content):
    """
    Parses a C file content and extracts function names and their implementation status.
    Status is:
    - 'error' if #error is found in the body.
    - 'todo' if #error is not found but TODO: or todo: is found.
    - 'check' otherwise.
    """
    # Pattern to match function definitions:
    # return_type function_name(args) {
    # We assume return type starts at the beginning of a line.
    pattern = r'^([\w\s\*]+)\s+(\w+)\s*\(([^)]*)\)\s*\{'
    KEYWORDS = {'if', 'while', 'for', 'switch', 'return'}
    
    functions = {}
    
    for match in re.finditer(pattern, content, re.MULTILINE):
        name = match.group(2)
        if name in KEYWORDS:
            continue
        start_pos = match.end()
        
        # Find matching brace to get function body
        brace_count = 1
        end_pos = start_pos
        while brace_count > 0 and end_pos < len(content):
            if content[end_pos] == '{':
                brace_count += 1
            elif content[end_pos] == '}':
                brace_count -= 1
            end_pos += 1
        
        body = content[start_pos:end_pos-1]
        
        status = 'check'
        if '#error' in body:
            status = 'error'
        elif 'TODO:' in body.upper():
            status = 'todo'
        
        functions[name] = status
        
    return functions

def main():
    parser = argparse.ArgumentParser(description="Check BSP implementation status.")
    parser.add_argument("--csv", action="store_true", help="Output as CSV instead of GUI")
    parser.add_argument("--transpose", action="store_true", help="Transpose columns and rows")
    args = parser.parse_args()

    # Find BSP folders relative to project root
    # We assume the script is in tools/ but run from project root.
    # If not, we try to find the root.
    project_root = Path(os.getcwd())
    bsp_root = project_root / "firmware" / "bsp"
    
    if not bsp_root.exists():
        # Maybe we are in tools/?
        if project_root.name == "tools":
            bsp_root = project_root.parent / "firmware" / "bsp"
        else:
            print(f"Error: Could not find firmware/bsp in {project_root}")
            sys.exit(1)
        
    bsps = []
    # Get all subdirectories in firmware/bsp except 'template'
    for d in bsp_root.iterdir():
        if d.is_dir() and d.name != "template" and (d / "bsp.c").exists():
            bsps.append(d.name)
    
    if not bsps:
        print("No BSPs with bsp.c found.")
        sys.exit(0)
        
    bsps.sort()
    
    data = {}
    all_functions = set()
    
    for bsp in bsps:
        bsp_file = bsp_root / bsp / "bsp.c"
        try:
            with open(bsp_file, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            funcs = get_bsp_functions(content)
            data[bsp] = funcs
            all_functions.update(funcs.keys())
        except Exception as e:
            print(f"Error reading {bsp_file}: {e}")
            data[bsp] = {}
        
    # Sorting functions: common first (count of appearances across BSPs), then by name
    func_counts = {}
    for func in all_functions:
        count = sum(1 for bsp in bsps if func in data[bsp])
        func_counts[func] = count
    
    sorted_functions = sorted(all_functions, key=lambda f: (-func_counts[f], f))
    
    if args.csv:
        output_csv(bsps, sorted_functions, data, args.transpose)
    else:
        try:
            output_gui(bsps, sorted_functions, data, args.transpose)
        except Exception as e:
            if "display" in str(e).lower() or "tk" in str(e).lower():
                print(f"Note: Could not start GUI ({e}). Falling back to CSV.")
                output_csv(bsps, sorted_functions, data, args.transpose)
            else:
                raise

def output_csv(bsps, functions, data, transpose):
    writer = csv.writer(sys.stdout)
    if not transpose:
        header = ["Function"] + bsps
        writer.writerow(header)
        for func in functions:
            row = [func]
            for bsp in bsps:
                status = data[bsp].get(func, None)
                if status == 'error': row.append("X")
                elif status == 'check': row.append("OK")
                elif status == 'todo': row.append("TODO")
                else: row.append("-")
            writer.writerow(row)
    else:
        header = ["BSP"] + functions
        writer.writerow(header)
        for bsp in bsps:
            row = [bsp]
            for func in functions:
                status = data[bsp].get(func, None)
                if status == 'error': row.append("X")
                elif status == 'check': row.append("OK")
                elif status == 'todo': row.append("TODO")
                else: row.append("-")
            writer.writerow(row)

def output_gui(bsps, functions, data, transpose):
    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError:
        print("tkinter not found. Please install python3-tk.")
        output_csv(bsps, functions, data, transpose)
        return

    root = tk.Tk()
    root.title("BSP Implementation Status")
    root.geometry("1000x800")

    # Outer frame to hold canvas and scrollbars
    outer_frame = ttk.Frame(root)
    outer_frame.pack(fill=tk.BOTH, expand=True)

    canvas = tk.Canvas(outer_frame)
    scrollbar_y = ttk.Scrollbar(outer_frame, orient="vertical", command=canvas.yview)
    scrollbar_x = ttk.Scrollbar(outer_frame, orient="horizontal", command=canvas.xview)
    scrollable_frame = ttk.Frame(canvas)

    scrollable_frame.bind(
        "<Configure>",
        lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
    )

    canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar_y.set, xscrollcommand=scrollbar_x.set)

    # Mouse wheel support
    def _on_mousewheel(event):
        canvas.yview_scroll(int(-1*(event.delta/120)), "units")
    
    canvas.bind_all("<MouseWheel>", _on_mousewheel)

    if not transpose:
        cols = bsps
        rows = functions
    else:
        cols = functions
        rows = bsps

    # Header row
    tk.Label(scrollable_frame, text="Name \\ BSP" if not transpose else "BSP \\ Name", 
             font=('Arial', 10, 'bold'), borderwidth=1, relief="solid", padx=5, pady=5).grid(row=0, column=0, sticky="nsew")
    for c, col_name in enumerate(cols):
        tk.Label(scrollable_frame, text=col_name, font=('Arial', 10, 'bold'), 
                 borderwidth=1, relief="solid", padx=5, pady=5).grid(row=0, column=c+1, sticky="nsew")

    # Data rows
    for r, row_name in enumerate(rows):
        tk.Label(scrollable_frame, text=row_name, font=('Arial', 10, 'bold'), 
                 borderwidth=1, relief="solid", padx=5, pady=2, anchor='e').grid(row=r+1, column=0, sticky="nsew")
        for c, col_name in enumerate(cols):
            if not transpose:
                status = data[col_name].get(row_name, None)
            else:
                status = data[row_name].get(col_name, None)
            
            text = "-"
            fg = "grey"
            if status == 'error':
                text = "✘"
                fg = "red"
            elif status == 'check':
                text = "✔"
                fg = "green"
            elif status == 'todo':
                text = "?"
                fg = "#D4A017" # Darker yellow/gold for visibility
            
            tk.Label(scrollable_frame, text=text, fg=fg, font=('Arial', 14, 'bold'),
                     borderwidth=1, relief="solid", width=4).grid(row=r+1, column=c+1, sticky="nsew")

    canvas.pack(side="left", fill="both", expand=True)
    scrollbar_y.pack(side="right", fill="y")
    scrollbar_x.pack(side="bottom", fill="x")

    root.mainloop()

if __name__ == "__main__":
    main()
