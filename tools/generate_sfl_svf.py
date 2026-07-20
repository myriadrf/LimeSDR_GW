#!/usr/bin/env python3

# Regenerate the Serial Flash Loader (SFL) configuration SVF for the LimeSDR-USB.
#
# The LimeSDR-USB flash SVF (bitstream/LimeSDR_USB/LimeSDR-USB_lms7_trx_flash.svf)
# is built at gateware build time by concatenating two parts:
#   1. an SFL *configuration* SVF, which loads the flash bridge into the FPGA
#      SRAM - this part is device-specific and constant (it does NOT depend on
#      the user gateware), so it is committed to the repository, and
#   2. a flash *operations* SVF (erase/program/verify) derived from the .jic,
#      which is regenerated on every build because it contains the bitstream.
#
# This script (re)generates part 1 from a Serial Flash Loader image shipped with
# the active Quartus installation. Run it only when the committed SFL SVF needs
# refreshing (e.g. after a Quartus major-version upgrade). openFPGALoader itself
# does not need Quartus, so end users flashing the committed *_flash.svf are not
# affected by their Quartus version.
#
# Usage:
#   python3 tools/generate_sfl_svf.py

import os
import shutil
import argparse
import subprocess

# The SFL image is selected by the device JTAG IDCODE (0x020F40DD, EP4CE40) and
# must match the version the .jic-derived flash operations expect.
DEFAULT_SFL_IMAGE = "sfl_enhanced_01_020f40dd.sof"
DEFAULT_OUTPUT    = "gateware/board_specific/limesdr_usb/sfl_ep4ce40_020f40dd.svf"

def find_quartus_sfl_image(name):
    """Locate a Serial Flash Loader image inside the active Quartus installation."""
    search_roots = []
    quartus_rootdir = os.environ.get("QUARTUS_ROOTDIR")
    if quartus_rootdir:
        search_roots.append(quartus_rootdir)
    quartus_cpf = shutil.which("quartus_cpf")
    if quartus_cpf:
        # <quartus>/bin/quartus_cpf -> <quartus>
        search_roots.append(os.path.dirname(os.path.dirname(quartus_cpf)))

    for root in search_roots:
        candidate = os.path.join(root, "common", "devinfo", "programmer", name)
        if os.path.exists(candidate):
            return candidate
    return None

def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    parser = argparse.ArgumentParser(
        description="Regenerate the LimeSDR-USB SFL configuration SVF from a Quartus installation.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--sfl-image", default=DEFAULT_SFL_IMAGE,
                        help="Serial Flash Loader image name to look up in the Quartus installation.")
    parser.add_argument("--output", default=os.path.join(repo_root, DEFAULT_OUTPUT),
                        help="Destination SVF file.")
    parser.add_argument("--frequency", default="12.0MHz", help="JTAG frequency passed to quartus_cpf.")
    parser.add_argument("--voltage",   default="3.3",     help="I/O voltage passed to quartus_cpf.")
    args = parser.parse_args()

    sfl_image = find_quartus_sfl_image(args.sfl_image)
    if sfl_image is None:
        raise SystemExit(
            f"Could not locate the Serial Flash Loader image '{args.sfl_image}' in the "
            "Quartus installation. Make sure Quartus is installed and QUARTUS_ROOTDIR "
            "is set (or quartus_cpf is on PATH)."
        )
    print(f"Using SFL image: {sfl_image}")

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    subprocess.run(
        ["quartus_cpf", "-c", "-q", args.frequency, "-g", args.voltage, "-n", "p", sfl_image, args.output],
        check=True,
    )
    print(f"Generated SFL configuration SVF: {args.output}")

if __name__ == "__main__":
    main()
