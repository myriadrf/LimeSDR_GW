#!/usr/bin/env python3

import os
import shutil
import subprocess

# Get script path
script_path = os.path.dirname(os.path.abspath(__file__))
curr_path   = os.getcwd()
print(f"Script Path: {script_path}")

# User editable variables
device                  = "LFE5U-45F"
bit_file_name           = "limesdr_mini_v2.bit"
golden_bit_file_name    = "limesdr_mini_v2_golden.bit" # Previous non-LiteX golden bitstream.
alternate_bit_file_name = "limesdr_mini_v2.bit"
mcs_output_file_name    = "limesdr_mini_v2.mcs"
impl_dir                = "build/limesdr_mini_v2/gateware"

# Copying .bit file from project directory
source_bit_file      = os.path.join(curr_path, impl_dir, bit_file_name)
destination_bit_file = os.path.join(script_path, bit_file_name)
print("Copying .bit file from project directory")
shutil.copyfile(source_bit_file, destination_bit_file)

# Creating required variables for ddtcmd
ddtcmd_device  = device
ddtcmd_if      = destination_bit_file
ddtcmd_golden  = os.path.join(script_path, golden_bit_file_name)
ddtcmd_of      = os.path.join(script_path, mcs_output_file_name)

# Launching Lattice Diamond Deployment Tool
print("Launching Lattice Diamond Deployment Tool 3.12, make sure it is added to system path")
try:
    subprocess.run([
        "ddtcmd",
        "-oft",       "-boot",
        "-dev",       ddtcmd_device,
        "-primary",   ddtcmd_if,
        "-format",    "int",
        "-flashsize", "128",
        "-golden",    ddtcmd_golden,
        "-goldenadd", "0x00140000",
        "-of",        ddtcmd_of
    ], check=True)
    print("ddtcmd execution completed successfully.")
except subprocess.CalledProcessError as e:
    print(f"Error occurred during ddtcmd execution: {e}")
