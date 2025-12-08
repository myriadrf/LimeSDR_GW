#!/usr/bin/env python3
#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0
#
# Test script for LimePSB-RPCM GPSDO via limeCSR shell command
#

import time
import argparse
import math
import subprocess
import sys

# Register Map (Physical Addresses) ----------------------------------------------------------------
# Base: 0xF0000000 + Offset: 0xB000 = 0xF000B000
BASE_ADDR = 0xF000B000

REG_MAP = {
    "ppsdo_enable":                 BASE_ADDR + 0x00,
    "ppsdo_config_one_s_target":    BASE_ADDR + 0x04,
    "ppsdo_config_one_s_tol":       BASE_ADDR + 0x08,
    "ppsdo_config_ten_s_target":    BASE_ADDR + 0x0C,
    "ppsdo_config_ten_s_tol":       BASE_ADDR + 0x10,
    "ppsdo_config_hundred_s_target":BASE_ADDR + 0x14,
    "ppsdo_config_hundred_s_tol":   BASE_ADDR + 0x18,
    "ppsdo_status_one_s_error":     BASE_ADDR + 0x1C,
    "ppsdo_status_ten_s_error":     BASE_ADDR + 0x20,
    "ppsdo_status_hundred_s_error": BASE_ADDR + 0x24,
    "ppsdo_status_dac_tuned_val":   BASE_ADDR + 0x28,
    "ppsdo_status_accuracy":        BASE_ADDR + 0x2C,
    "ppsdo_status_pps_active":      BASE_ADDR + 0x30,
    "ppsdo_status_state":           BASE_ADDR + 0x34,
}

# Control bit fields
CONTROL_EN_OFFSET      = 0
CONTROL_EN_SIZE        = 1
CONTROL_CLK_SEL_OFFSET = 1
CONTROL_CLK_SEL_SIZE   = 1

# Helper function to set a field within a register value.
def set_field(reg_value, offset, size, value):
    mask = ((1 << size) - 1) << offset
    return (reg_value & ~mask) | ((value << offset) & mask)

# GPSDODriver --------------------------------------------------------------------------------------
class GPSDODriver:
    """
    Driver for LimePSB-RPCM GPSDO using 'limeCSR' shell command.
    """
    def __init__(self):
        # Check if limeCSR is available
        try:
            subprocess.run(["limeCSR", "--help"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            print("Error: 'limeCSR' command not found. Please ensure it is installed and in your PATH.")
            sys.exit(1)

    def _exec_read(self, addr):
        """Executes: limeCSR read -s<ADDR64>"""
        addr_str = f"{addr:016x}"
        cmd = ["limeCSR", "read", f"-s{addr_str}"]

        try:
            result = subprocess.check_output(cmd, text=True)
        except subprocess.CalledProcessError as e:
            print(f"Error reading CSR: {e}")
            return 0

        # Parse output: 0x00000000f000b004 0x00000000018cba80
        lines = result.strip().split('\n')
        if not lines:
            return 0

        last_line = lines[-1].strip()
        parts = last_line.split()

        if len(parts) >= 2:
            try:
                return int(parts[1], 16)
            except ValueError:
                pass
        return 0

    def _exec_write(self, addr, value):
        """Executes: limeCSR write -s<ADDR64><VALUE64>"""
        addr_str = f"{addr:016x}"
        val_str = f"{value:016x}"
        arg_str = f"-s{addr_str}{val_str}"

        cmd = ["limeCSR", "write", arg_str]

        try:
            subprocess.check_call(cmd, stdout=subprocess.DEVNULL)
        except subprocess.CalledProcessError as e:
            print(f"Error writing CSR: {e}")

    def read_register(self, name):
        if name not in REG_MAP:
            raise KeyError(f"Register {name} not found in map")
        return self._exec_read(REG_MAP[name])

    def write_register(self, name, value):
        if name not in REG_MAP:
            raise KeyError(f"Register {name} not found in map")
        self._exec_write(REG_MAP[name], value)

    def get_signed_32bit(self, name):
        value = self.read_register(name)
        value = value & 0xFFFFFFFF
        if value & (1 << 31):  # Sign extend if negative
            value -= (1 << 32)
        return value

    def get_1s_error(self):
        return self.get_signed_32bit("ppsdo_status_one_s_error")

    def get_10s_error(self):
        return self.get_signed_32bit("ppsdo_status_ten_s_error")

    def get_100s_error(self):
        return self.get_signed_32bit("ppsdo_status_hundred_s_error")

    def get_dac_value(self):
        return self.read_register("ppsdo_status_dac_tuned_val")

    def get_status(self):
        state    = self.read_register("ppsdo_status_state")
        accuracy = self.read_register("ppsdo_status_accuracy")
        tpulse   = self.read_register("ppsdo_status_pps_active")

        state_str = "Coarse Tune" if state == 0 else "Fine Tune" if state == 1 else f"Unknown ({state})"
        accuracy_str = ['Disabled/Lowest', '1s Tune', '2s Tune', '3s Tune (Highest)'][accuracy] if accuracy < 4 else f"Unknown ({accuracy})"

        return {
            "state": state_str,
            "accuracy": accuracy_str,
            "tpulse_active": bool(tpulse)
        }

    def get_enabled(self):
        control = self.read_register("ppsdo_enable")
        return bool(control & 0x0001)

    def set_enabled(self, enable):
        control = self.read_register("ppsdo_enable")
        control = set_field(control, CONTROL_EN_OFFSET, CONTROL_EN_SIZE, 1 if enable else 0)
        self.write_register("ppsdo_enable", control)

    def close(self):
        pass

# Test Functions -----------------------------------------------------------------------------------
def run_monitoring(driver, num_dumps=0, delay=1.0, banner_interval=10):
    # Fixed widths and alignment for cleaner output
    cols = [
        ("Dump",       4,  ">"),
        ("Enabled",    7,  ">"),
        ("1s Error",   8,  ">"),
        ("10s Error",  9,  ">"),
        ("100s Error", 10, ">"),
        ("DAC Value",  9,  ">"),
        ("State",      12, "<"), # Left align text
        ("Accuracy",   17, "<"), # Left align text
        ("TPulse",     6,  ">")
    ]

    def print_header():
        parts = []
        for name, width, align in cols:
            parts.append(f"{name:{align}{width}}")
        print(" | ".join(parts))

    print("Monitoring GPSDO regulation loop (press Ctrl+C to stop):")
    print_header()

    dump_count = 0
    try:
        while num_dumps == 0 or dump_count < num_dumps:
            enabled = driver.get_enabled()
            error_1s = driver.get_1s_error()
            error_10s = driver.get_10s_error()
            error_100s = driver.get_100s_error()
            dac = driver.get_dac_value()
            status = driver.get_status()

            dac_str = f"0x{dac:04X}"

            print(f"{dump_count + 1:4d} | "
                  f"{str(enabled):>7} | "
                  f"{error_1s:8d} | "
                  f"{error_10s:9d} | "
                  f"{error_100s:10d} | "
                  f"{dac_str:>9} | "
                  f"{status['state']:<12} | "
                  f"{status['accuracy']:<17} | "
                  f"{str(status['tpulse_active']):>6}")

            dump_count += 1
            if dump_count % banner_interval == 0:
                print_header()

            if num_dumps == 0 or dump_count < num_dumps:
                time.sleep(delay)
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")

def reset_gpsdo(driver, reset_delay=2.0):
    print("Resetting GPSDO...")
    driver.set_enabled(False)
    time.sleep(reset_delay)
    driver.set_enabled(True)
    print("GPSDO reset complete (re-enabled).")

def enable_gpsdo(driver, clk_freq_mhz=30.72, ppm=0.1):
    freq = clk_freq_mhz * 1e6

    target_1s   = int(freq)
    target_10s  = int(10 * freq)
    target_100s = int(100 * freq)

    # 1. Calculate raw tolerance based on PPM
    raw_tol_1s = math.ceil(freq * ppm / 1e6)

    # 2. Enforce minimum tolerance of 2 Hz to prevent glitches
    if raw_tol_1s < 2:
        tol_1s_hz = 2
        clamp_msg = " (Notice: 1s tolerance increased to min 2Hz)"
    else:
        tol_1s_hz = raw_tol_1s
        clamp_msg = ""

    # 3. Scale for 10s and 100s
    tol_10s_hz  = raw_tol_1s * 10
    tol_100s_hz = raw_tol_1s * 100

    driver.write_register("ppsdo_config_one_s_target",   target_1s)
    driver.write_register("ppsdo_config_one_s_tol",      tol_1s_hz)
    driver.write_register("ppsdo_config_ten_s_target",   target_10s)
    driver.write_register("ppsdo_config_ten_s_tol",      tol_10s_hz)
    driver.write_register("ppsdo_config_hundred_s_target", target_100s)
    driver.write_register("ppsdo_config_hundred_s_tol",    tol_100s_hz)

    clk_sel = 1 if math.isclose(clk_freq_mhz, 10.0) else 0
    control = set_field(0, CONTROL_CLK_SEL_OFFSET, CONTROL_CLK_SEL_SIZE, clk_sel)
    control = set_field(control, CONTROL_EN_OFFSET, CONTROL_EN_SIZE, 1)
    driver.write_register("ppsdo_enable", control)

    print(f"GPSDO enabled: CLK_SEL={clk_sel} ({clk_freq_mhz}MHz), {ppm}ppm tolerance{clamp_msg}")
    print(f"    1s tol={tol_1s_hz}Hz, 10s={tol_10s_hz}Hz, 100s={tol_100s_hz}Hz")

def disable_gpsdo(driver):
    driver.write_register("ppsdo_enable", 0x0000)
    print("GPSDO disabled.")

# Main ----------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="GPSDO Test Script (limeCSR CLI)")
    parser.add_argument("--check", action="store_true", help="Run monitoring mode")
    parser.add_argument("--reset", action="store_true", help="Reset GPSDO")
    parser.add_argument("--enable", action="store_true", help="Configure and enable GPSDO")
    parser.add_argument("--disable", action="store_true", help="Disable GPSDO")
    parser.add_argument("--num", default=0, type=int, help="Number of iterations")
    parser.add_argument("--delay", default=1.0, type=float, help="Delay between iterations")
    parser.add_argument("--banner", default=10, type=int, help="Banner repeat interval")
    parser.add_argument("--reset-delay", default=2.0, type=float, help="Delay after disable before re-enable")
    parser.add_argument("--clk-freq", default=30.72, type=float, help="Clock frequency in MHz")
    parser.add_argument("--ppm", default=0.1, type=float, help="Tolerance in ppm")
    args = parser.parse_args()

    driver = GPSDODriver()
    try:
        if args.enable:
            enable_gpsdo(driver, clk_freq_mhz=args.clk_freq, ppm=args.ppm)

        if args.disable:
            disable_gpsdo(driver)

        if args.reset:
            reset_gpsdo(driver, reset_delay=args.reset_delay)

        if args.check:
            run_monitoring(driver, num_dumps=args.num, delay=args.delay, banner_interval=args.banner)
    finally:
        driver.close()

if __name__ == "__main__":
    main()
