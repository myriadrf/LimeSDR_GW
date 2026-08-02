#!/usr/bin/env python3

import os
import sys
import argparse
import subprocess
import time
from pathlib import Path

def discover_targets():
    """Find all target scripts in boards/targets/"""
    targets_dir = Path("boards/targets")
    targets = {}
    if not targets_dir.exists():
        print(f"Error: {targets_dir} directory not found.")
        return targets
        
    for f in targets_dir.glob("*.py"):
        if f.name == "__init__.py":
            continue
        # Use stem as target name
        targets[f.stem] = str(f)
    return dict(sorted(targets.items()))

def detect_gold_flag(target_path):
    """Check if the target script supports --gold or --golden"""
    try:
        env = os.environ.copy()
        env["PYTHONPATH"] = os.getcwd() + (":" + env.get("PYTHONPATH", "") if env.get("PYTHONPATH") else "")
        result = subprocess.run(
            [sys.executable, target_path, "--help"],
            capture_output=True,
            text=True,
            check=False,
            env=env
        )
        # Search for flags in help output
        if "--golden" in result.stdout:
            return "--golden"
        if "--gold" in result.stdout:
            return "--gold"
    except Exception as e:
        print(f"Warning: Could not detect gold flag for {target_path}: {e}")
    return None

def run_command(cmd, log_file, dry_run=False):
    """Execute a command and redirect output to log_file"""
    if dry_run:
        return True
    
    try:
        env = os.environ.copy()
        env["PYTHONPATH"] = os.getcwd() + (":" + env.get("PYTHONPATH", "") if env.get("PYTHONPATH") else "")
        with open(log_file, "w") as f:
            f.write(f"Executing: {' '.join(cmd)}\n")
            f.write("-" * 40 + "\n")
            f.flush()
            result = subprocess.run(
                cmd,
                stdout=f,
                stderr=subprocess.STDOUT,
                text=True,
                env=env
            )
        return result.returncode == 0
    except Exception as e:
        print(f"  [ERROR] {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="Rebuild LimeSDR Gateware targets sequentially.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--targets", nargs="+", help="Specific targets to rebuild (e.g. limesdr_usb). Builds all if omitted.")
    parser.add_argument("--logs-dir", default="build_logs", help="Directory where build logs will be stored.")
    parser.add_argument("--gold", action="store_true", help="Also build golden images for targets that support it.")
    parser.add_argument("--gold-first", action="store_true", help="Build all golden images before starting regular builds (requires --gold).")
    parser.add_argument("--dry-run", action="store_true", help="Show commands that would be executed without running them.")

    args = parser.parse_args()

    if args.gold_first and not args.gold:
        print("Error: --gold-first requires --gold flag.")
        sys.exit(1)

    available_targets = discover_targets()
    
    if args.targets:
        selected_targets = {}
        for t in args.targets:
            if t in available_targets:
                selected_targets[t] = available_targets[t]
            else:
                print(f"Error: Target '{t}' not found in boards/targets/")
                print(f"Available targets: {', '.join(available_targets.keys())}")
                sys.exit(1)
    else:
        selected_targets = available_targets

    if not selected_targets:
        print("No targets found in boards/targets/")
        sys.exit(0)

    print(f"Found {len(selected_targets)} target(s) to rebuild.")
    if not args.dry_run:
        os.makedirs(args.logs_dir, exist_ok=True)
        print(f"Logs will be stored in: {os.path.abspath(args.logs_dir)}")

    # Detect gold flags for selected targets
    target_gold_flags = {}
    if args.gold:
        print("Detecting gold build support...")
        for name, path in selected_targets.items():
            flag = detect_gold_flag(path)
            if flag:
                target_gold_flags[name] = flag
                print(f"  {name}: Supports {flag}")
            else:
                print(f"  {name}: No gold build support detected.")

    # Generate Build Plan
    build_plan = []
    if args.gold and args.gold_first:
        # Pass 1: All Golden
        for name, path in selected_targets.items():
            if name in target_gold_flags:
                build_plan.append((name, path, "golden", target_gold_flags[name]))
        # Pass 2: All Regular
        for name, path in selected_targets.items():
            build_plan.append((name, path, "regular", None))
    else:
        # Sequential: Golden then Regular per board
        for name, path in selected_targets.items():
            if args.gold and name in target_gold_flags:
                build_plan.append((name, path, "golden", target_gold_flags[name]))
            build_plan.append((name, path, "regular", None))

    results = []
    
    print("\nStarting build sequence...")
    for i, (name, path, btype, gold_flag) in enumerate(build_plan, 1):
        cmd = [sys.executable, path, "--build"]
        log_name = name
        if btype == "golden":
            cmd.append(gold_flag)
            log_name += "_golden"
        
        log_file = os.path.join(args.logs_dir, f"{log_name}.log")
        
        print(f"[{i}/{len(build_plan)}] Building {name} ({btype})...")
        if args.dry_run:
            print(f"  [DRY-RUN] {' '.join(cmd)}")
        
        start_time = time.time()
        success = run_command(cmd, log_file, args.dry_run)
        elapsed = time.time() - start_time
        
        status_msg = "SUCCESS" if success else "FAILED"
        if args.dry_run:
            status_msg = "SKIP"
        
        print(f"  Result: {status_msg} ({elapsed:.1f}s)")
        
        results.append({
            "target": name,
            "type": btype,
            "success": success,
            "elapsed": elapsed
        })

    # Summary Table
    print("\n" + "="*70)
    print(f"{'Target':<25} | {'Type':<10} | {'Status':<10} | {'Duration':<10}")
    print("-" * 70)
    for res in results:
        status = "SUCCESS" if res["success"] else "FAILED"
        if args.dry_run:
            status = "DRY-RUN"
        
        duration = f"{res['elapsed']:.1f}s"
        print(f"{res['target']:<25} | {res['type']:<10} | {status:<10} | {duration:<10}")
    print("="*70)

    failed = [r for r in results if not r["success"] and not args.dry_run]
    if failed:
        print(f"\nCompleted with {len(failed)} failure(s).")
        sys.exit(1)
    else:
        print("\nAll builds completed successfully.")

if __name__ == "__main__":
    main()
