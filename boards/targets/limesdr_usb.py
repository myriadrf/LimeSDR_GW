#!/usr/bin/env python3

#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from boards.platforms import limesdr_usb_platform as limesdr_usb

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

# Constants ----------------------------------------------------------------------------------------

# TODO: Define constants (FIFO sizes, etc.)

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst      = Signal()
        self.cd_sys   = ClockDomain()

        # # #

        # TODO: Implement Clock and Reset Generation for Cyclone IV.
        # Skeleton:
        # self.clk = platform.request("???")
        # self.pll = CycloneIVPLL(platform)
        # self.pll.register_clkin(self.clk, ???)
        # self.pll.create_clkout(self.cd_sys, sys_clk_freq)
        # self.specials += AsyncResetSynchronizer(self.cd_sys, self.rst)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=50e6, **kwargs):
        platform = limesdr_usb.Platform()

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on LimeSDR-USB", **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # TODO: Add modules and peripherals:
        # - FX3 (USB interface)
        # - LMS7002M (RF transceiver)
        # - SPI, I2C, GPIOs, etc.

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LimeSDR USB LiteX Gateware.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Build/Load/Utilities.
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    parser.add_argument("--cable", default="usb-blaster", help="JTAG cable.")

    # SoC parameters.
    parser.add_argument("--with-bios", action="store_true", help="Enable LiteX BIOS.")

    args = parser.parse_args()

    # Build SoC.
    for run in range(2):
        prepare = (run == 0)
        build   = ((run == 1) & args.build)
        
        # SoC.
        soc = BaseSoC(
            with_bios    = args.with_bios,
            cpu_firmware = None if prepare else "firmware/firmware.bin"
        )
        
        # Builder.
        builder = Builder(soc, csr_csv="csr.csv", bios_console="lite", libc_mode="full")
        builder.build(run=build)
        
        # Firmware build.
        if prepare:
            linker = {
                True  : "linker_main_ram.ld",
                False : "linker_rom.ld",
            }[args.with_bios]
            
            # Create a makefile fragment with board specific variables
            env_mak = os.path.join("firmware", "env.mak")
            if os.path.exists(env_mak):
                os.remove(env_mak)
            with open(env_mak, "w") as f:
                f.write(f"BUILD_DIR={builder.output_dir}\n")
                f.write(f"TARGET=LIMESDR_USB\n")
                f.write(f"LINKER={linker}\n")
                f.write("BSP_PROJECT_DIR=bsp/LimeSDR_USB\n")
            
            # Note: Do not run make here yet as it's a skeleton.

    if args.load:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".sof"))

if __name__ == "__main__":
    main()
