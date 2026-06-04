#!/usr/bin/env python3

#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os
import argparse

from gateware.LimeDFB.FX3.src.FX3 import FX3
from gateware.helpers import write_module_hierarchy_json
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

        self.fx3_pclk = platform.request("FX3_PCLK")
        self.comb += self.cd_sys.clk.eq(self.fx3_pclk)
        # FX3 PCLK runs at 100MHz
        platform.add_period_constraint(self.cd_sys.clk, 1e9/100e6)

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
    def __init__(self,
                 sys_clk_freq= 100e6,
                 with_bios         = False,
                 gold_img          = False,
                 cpu_firmware      = None):
        platform = limesdr_usb.Platform()
        platform.name        = "limesdr_usb"
        platform.vhd2v_force = False

        if with_bios:
            integrated_rom_size      = 0x6800
            integrated_rom_init      = []
            integrated_main_ram_size = 0x6800
            integrated_main_ram_init = [] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
        else:
            integrated_rom_size      = 0x6800
            integrated_rom_init      = [0] if cpu_firmware is None else get_mem_data(cpu_firmware, endianness="little")
            integrated_main_ram_size = 0
            integrated_main_ram_init = []

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident                    = "LiteX SoC on LimeSDR-USB",
            ident_version            = True,
            cpu_type                 = "vexriscv",
            cpu_variant              = "minimal",
            integrated_rom_size      = integrated_rom_size,
            integrated_rom_init      = integrated_rom_init,
            integrated_sram_size     = 0x2000,
            integrated_main_ram_size = integrated_main_ram_size,
            integrated_main_ram_init = integrated_main_ram_init,
            with_uart                = False, #for now
            # with_uartbone            = with_uartbone,
            # uart_name                = {True: "crossover", False:"serial"}[with_uartbone],
        )

        # 1 for CSR
        # 2 for FTDI
        # 3 for FX3
        self.add_constant("LMS64C_METHOD",3)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # FX3
        self.FX3 = FX3(platform=platform,
                       pads=platform.request("FX3"))

        # TODO: Add modules and peripherals:
        # - FX3 (USB interface)
        # - LMS7002M (RF transceiver)
        # - SPI, I2C, GPIOs, etc.

    # Utils
    def print_soc_hierarchy_json(self, outfile=None):
        """Generate the SoC submodule hierarchy and write it as JSON to soc_structure.json.
        The filename is constant. No terminal printing.
        """
        write_module_hierarchy_json(self, outfile="soc_structure.json", name="SoC")


    def generate_documentation(self, build_name, build_html, **kwargs):
        from litex.soc.doc import generate_docs
        generate_docs(self, "docs/docs/{}/litex_doc".format(build_name),
            project_name = "{}".format(build_name),
            author       = "Lime Microsystems")
        if build_html:
            os.system("sphinx-build -M html docs/docs/{}/litex_doc docs/docs/{}/litex_doc/_build".format(build_name, build_name))

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LimeSDR USB LiteX Gateware.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Build/Load/Utilities.
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    parser.add_argument("--cable", default="usb-blaster", help="JTAG cable.")

    # SoC parameters.
    parser.add_argument("--with-bios", action="store_true", help="Enable LiteX BIOS.")

    # Introspection.
    parser.add_argument("--no-soc-json",    action="store_true", help="Disable automatic SoC hierarchy JSON generation.")
    parser.add_argument("--doc",    action="store_true", help="Generate SOC ducumentation")

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

        # Always generate SoC hierarchy JSON during prepare pass unless disabled.
        if prepare and not args.no_soc_json:
            soc.print_soc_hierarchy_json()
        
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

    # Generate Litex Documentation files and if --doc option is used build also
    build_name = soc.build_name.replace("_", "-")
    soc.generate_documentation(build_name, build_html=args.doc)

if __name__ == "__main__":
    main()
