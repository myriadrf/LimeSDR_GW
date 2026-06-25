#!/usr/bin/env python3

#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os
import argparse
import shutil
import subprocess

from gateware.LimeDFB.FX3.src.FX3 import FX3
from gateware.LimeTop import LimeTop
from gateware.board_specific.limesdr_usb.PSS_LimeSDR_Usb import PSS_LimeSDR_Usb
from gateware.helpers import write_module_hierarchy_json
from gateware.Revision import *
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

        # Safety feature. Currently there is no support for other frequencies.
        assert sys_clk_freq == 100e6

        self.cd_sys   = ClockDomain()
        self.fx3_pclk = platform.request("FX3_PCLK")
        # FX3 PCLK runs at 100MHz
        platform.add_period_constraint(self.fx3_pclk, 1e9/100e6)
        self.comb += self.cd_sys.clk.eq(self.fx3_pclk)

        self.ext_gnd = Signal()
        self.ext_gnd = platform.request("EXT_GND")
        self.specials += AsyncResetSynchronizer(self.cd_sys,self.ext_gnd)

        self.cd_lmk  = ClockDomain()
        self.clk_lmk = platform.request("LMK_CLK")
        platform.add_period_constraint(self.clk_lmk, 1e9 / 30.72e6)
        self.comb += self.cd_lmk.clk.eq(self.clk_lmk)

        self.cd_si0 = ClockDomain()
        self.clk_si0 = platform.request("SI_CLK", 0)
        platform.add_period_constraint(self.clk_si0, 1e9 / 250e6)
        self.comb += self.cd_si0.clk.eq(self.clk_si0)

        self.cd_si1 = ClockDomain()
        self.clk_si1 = platform.request("SI_CLK", 1)
        platform.add_period_constraint(self.clk_si1, 1e9 / 250e6)
        self.comb += self.cd_si1.clk.eq(self.clk_si1)

        self.cd_si2 = ClockDomain()
        self.clk_si2 = platform.request("SI_CLK", 2)
        platform.add_period_constraint(self.clk_si2, 1e9 / 250e6)
        self.comb += self.cd_si2.clk.eq(self.clk_si2)

        self.cd_si3 = ClockDomain()
        self.clk_si3 = platform.request("SI_CLK", 3)
        platform.add_period_constraint(self.clk_si3, 1e9 / 250e6)
        self.comb += self.cd_si3.clk.eq(self.clk_si3)

        # No SI CLK 4

        self.cd_si5 = ClockDomain()
        self.clk_si5 = platform.request("SI_CLK", 5)
        platform.add_period_constraint(self.clk_si5, 1e9 / 250e6)
        self.comb += self.cd_si5.clk.eq(self.clk_si5)

        self.cd_si6 = ClockDomain()
        self.clk_si6 = platform.request("SI_CLK", 6)
        platform.add_period_constraint(self.clk_si6, 1e9 / 250e6)
        self.comb += self.cd_si6.clk.eq(self.clk_si6)

        self.cd_si7 = ClockDomain()
        self.clk_si7 = platform.request("SI_CLK", 7)
        platform.add_period_constraint(self.clk_si7, 1e9 / 250e6)
        self.comb += self.cd_si7.clk.eq(self.clk_si7)

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
        platform.add_platform_command("set_global_assignment -name VHDL_INPUT_VERSION VHDL_2008")

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
                       pads=platform.request("FX3"),
                       vendor="altera",
                       EP01_0_rwidth = 64,
                       EP01_1_rwidth = 64,
                       EP81_wwidth   = 64
                       )

        # LMS SPI -----------------------------------------------------------------------------------
        # LMS spi declared outside PSS, because the current firmware driver expects that
        self.add_spi_master(name="spimaster", pads=platform.request("FPGA_SPI0"), data_width=32, spi_clk_freq=1e6)

        # PSS (Peripheral Support Subsystem)
        self.pss = PSS_LimeSDR_Usb(self, platform, sys_clk_freq)

        # LimeTop -----------------------------------------------------------------------------------
        self.limetop  = LimeTop(self,
                                platform             = platform,
                                vendor               ="altera",
                                family               = "cycloneIV",
                                double_channels_mode = False,
                                one_chnl             = False,
                                LMS_DIQ_WIDTH        = 12,
                                sink_width           = 64,
                                sink_clk_domain      = "sys",
                                source_width         = 64,
                                source_clk_domain    = "sys",
                                rx_sys_clk_domain    = "sys",
                                rx_fixed_packet_size = True, # TODO: check
                                TX_N_BUFF            = 5,
                                TX_MAX_PCT_SIZE      = 4096,
                                tx_buffer_size       = 512, #TX buffer acts as CDC, so a minimum of 512 (4 cycles of 128bit) is required to instantiate the async FIFO

                                with_lms7002         = True,
                                # These clocks are only used if with_lms7002 is False
                                phy_tx_source_clk    = "sys",
                                phy_tx_sink_width    = 128,
                                phy_rx_sink_clk      = "sys",
                                phy_rx_sink_width    = 128,
                                with_rx_tx_top       = True,
                                fft_pts              = 512,     #Changing FFT points requires FFT src rebuild, use rebuild_fft_rtl=True
                                rebuild_fft_rtl      = False,
                                with_fft             = False,

                                # FPGACFG.
                                board_id             = 0x0011,
                                major_rev            = MajorRevision,
                                compile_rev          = CompileRevision,
                                revision_pads        = platform.request("revision"),

                                with_event_manager   = False,#True,
                                with_clk_cfg_irq     = False,#True,
                                soc_has_timesource   = False,
                                )

        self.comb += [
            self.FX3.data_source.connect  (self.limetop.sink),
            self.limetop.source.connect   (self.FX3.data_sink),
            self.FX3.data_sink_clr.eq     (~self.limetop.fpgacfg.rx_en),
            self.FX3.data_source0_clr.eq  (~self.limetop.fpgacfg.rx_en),
            self.FX3.data_source1_clr.eq  (~self.limetop.fpgacfg.rx_en),
        ]

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
            
            os.system(f"cd firmware && make clean all")

    if args.build:
        output_location = "bitstream/LimeSDR_USB"
        prefix = "LimeSDR-USB_lms7_trx"
        sof_file = os.path.join(builder.gateware_dir, "limesdr_usb.sof")

        os.makedirs(output_location, exist_ok=True)
        print(f"Copying limesdr_usb.sof to {output_location}...")
        shutil.copyfile(sof_file, os.path.join(output_location, prefix + ".sof"))

        for fmt in ["rbf", "jic", "pof"]:
            print(f"Generating {fmt.upper()} file...")
            try:
                subprocess.run(["quartus_cpf", "-c", f"gateware/limesdr_usb_{fmt}.cof"], check=True)
            except Exception as e:
                print(f"Error generating {fmt.upper()}: {e}")

        print("Bitstream generation completed.")

    if args.load:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".sof"))

    # Generate Litex Documentation files and if --doc option is used build also
    build_name = soc.build_name.replace("_", "-")
    soc.generate_documentation(build_name, build_html=args.doc)

if __name__ == "__main__":
    main()
