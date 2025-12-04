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
import re

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

# Status bit fields.
STATUS_STATE_OFFSET = 0
STATUS_STATE_SIZE   = 4
STATUS_ACCURACY_OFFSET = 4
STATUS_ACCURACY_SIZE   = 4
STATUS_TPULSE_OFFSET = 8
STATUS_TPULSE_SIZE   = 1

# Control bit fields
CONTROL_EN_OFFSET      = 0
CONTROL_EN_SIZE        = 1
CONTROL_CLK_SEL_OFFSET = 1
CONTROL_CLK_SEL_SIZE   = 1

# Helper function to get a field from a register value.
def get_field(reg_value, offset, size):
    mask = ((1 << size) - 1) << offset
    return (reg_value & mask) >> offset

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
        # Format address as 16 chars (64-bit) hex, lowercase
        addr_str = f"{addr:016x}"
        cmd = ["limeCSR", "read", f"-s{addr_str}"]

        try:
            result = subprocess.check_output(cmd, text=True)
        except subprocess.CalledProcessError as e:
            print(f"Error reading CSR: {e}")
            return 0

        # Parse output:
        # CSR data read out (address | value):
        # 0x00000000f000b004 0x00000000018cba80

        # We look for the hex pattern in the last line
        lines = result.strip().split('\n')
        if not lines:
            return 0

        last_line = lines[-1].strip()
        parts = last_line.split()

        if len(parts) >= 2:
            try:
                # The value is the second element
                return int(parts[1], 16)
            except ValueError:
                pass
        return 0

    def _exec_write(self, addr, value):
        """Executes: limeCSR write -s<ADDR64><VALUE64>"""
        addr_str = f"{addr:016x}"
        val_str = f"{value:016x}"
        # Combine strictly as requested: -s[ADDR][VALUE]
        arg_str = f"-s{addr_str}{val_str}"

        cmd = ["limeCSR", "write", arg_str]

        try:
            subprocess.check_call(cmd, stdout=subprocess.DEVNULL)
        except subprocess.CalledProcessError as e:
            print(f"Error writing CSR: {e}")

    def read_register(self, name):
        """Read a register by its name using the REG_MAP."""
        if name not in REG_MAP:
            raise KeyError(f"Register {name} not found in map")
        return self._exec_read(REG_MAP[name])

    def write_register(self, name, value):
        """Write a register by its name using the REG_MAP."""
        if name not in REG_MAP:
            raise KeyError(f"Register {name} not found in map")
        self._exec_write(REG_MAP[name], value)

    def get_signed_32bit(self, name):
        """Get signed 32-bit value from a register."""
        value = self.read_register(name)
        # Mask to 32-bit just in case the 64-bit return has garbage
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
        pass # Nothing to close for shell commands

# Test Functions -----------------------------------------------------------------------------------
def run_monitoring(driver, num_dumps=0, delay=1.0, banner_interval=10):
    header = "Dump | Enabled | 1s Error | 10s Error | 100s Error | DAC Value | State | Accuracy | TPulse"
    print("Monitoring GPSDO regulation loop (press Ctrl+C to stop):")
    print(header)
    dump_count = 0
    try:
        while num_dumps == 0 or dump_count < num_dumps:
            enabled = driver.get_enabled()
            error_1s = driver.get_1s_error()
            error_10s = driver.get_10s_error()
            error_100s = driver.get_100s_error()
            dac = driver.get_dac_value()
            status = driver.get_status()

            print(f"{dump_count + 1:4d} | {str(enabled):7} | {error_1s:8d} | {error_10s:9d} | {error_100s:10d} | 0x{dac:04X} | {status['state']:12} | {status['accuracy']:17} | {str(status['tpulse_active']):6}")
            dump_count += 1

            if dump_count % banner_interval == 0:
                print(header)

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

    tol_1s_hz   = math.ceil(freq * ppm / 1e6)
    tol_10s_hz  = tol_1s_hz * 10
    tol_100s_hz = tol_1s_hz * 100

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

    print(f"GPSDO enabled: CLK_SEL={clk_sel} ({clk_freq_mhz}MHz), {ppm}ppm tolerance "
          f"(1s tol={tol_1s_hz}Hz, 10s={tol_10s_hz}Hz, 100s={tol_100s_hz}Hz).")

def disable_gpsdo(driver):
    driver.write_register("ppsdo_enable", 0x0000)
    print("GPSDO disabled.")

# Main ----------------------------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="GPSDO Test Script (limeCSR CLI)")
    # Host/Port removed as they are irrelevant for local limeCSR command
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
