#!/usr/bin/env python3

#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

# Build/Use:
# ./limesdr_mini_v1.py --csr-csv=csr.csv --build --load

import math

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from boards.platforms import limesdr_mini_v1_platform as limesdr_mini_v1

from litex.soc.cores.led import LedChaser

from litex.soc.integration.soc      import SoCRegion

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from gateware.max10_dual_cfg.max10_dual_cfg import Max10DualCfg

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst      = Signal()
        self.cd_por   = ClockDomain()
        self.cd_sys   = ClockDomain()

        # # #

        # Clk.
        self.ft_clk = platform.request("FT_CLK")
        self.lmk_clk= platform.request("LMK_CLK")

        # Power-on-Clk/Rst.
        por_count = Signal(4)
        por_done  = Signal()
        por_count.attr.add("keep")
        por_done.attr.add("keep")
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)
        self.comb += por_done.eq(Reduce("AND", por_count))
        self.sync.por += por_count.eq(Cat(Constant(1, 1), por_count[0:3]))

        # Sys Clk/Rst.
        self.comb += self.cd_sys.clk.eq(self.lmk_clk)
        self.specials += AsyncResetSynchronizer(self.cd_sys, self.rst | ~por_done)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCMini):
    def __init__(self, sys_clk_freq=80e6, toolchain="quartus",
        **kwargs):

        # Platform ---------------------------------------------------------------------------------
        platform             = limesdr_mini_v1.Platform(toolchain=toolchain)
        platform.name        = "limesdr_mini_v1_blink"
        platform.add_platform_command("set_global_assignment -name VHDL_INPUT_VERSION VHDL_2008") # Enable VHDL-2008 support.
        platform.add_platform_command("set_global_assignment -name INTERNAL_FLASH_UPDATE_MODE \"DUAL IMAGES\"") # Required to access internal flash

        # SoCCore ----------------------------------------------------------------------------------

        SoCMini.__init__(self, platform, sys_clk_freq,
            ident                    = "LiteX SoC on LimeSDR-Mini-V2",
            ident_version            = True,
        )

        self.add_constant("LIMESDR_MINI_V1")

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # Max10 Dual Cfg ---------------------------------------------------------------------------
        self.dual_cfg = Max10DualCfg(platform)
        dual_cfg_region = SoCRegion(
                origin = 0x900000,
                size   = 0x100,
                mode   = "rwx")
        self.bus.add_slave("dual_cfg", self.dual_cfg.bus, dual_cfg_region)

        leds_g = Signal(4)
        leds_r = Signal(4)
        self.comb += platform.request_all("FPGA_LED1_G").eq(~leds_g)
        self.comb += platform.request_all("FPGA_LED1_R").eq(~leds_r)

        self.leds = LedChaser(Cat(leds_g, leds_r), sys_clk_freq)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LimeSDR-Mini-V2 LiteX Gateware.", formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Build/Load/Utilities.
    parser.add_argument("--build",     action="store_true", help="Build bitstream.")
    parser.add_argument("--toolchain", default="quartus",   help="FPGA toolchain.", choices=["quartus"])
    parser.add_argument("--load",      action="store_true", help="Load bitstream.")
    parser.add_argument("--flash",     action="store_true", help="Flash bitstream.")
    parser.add_argument("--cable",     default="ft2232",    help="JTAG cable.")

    args = parser.parse_args()

    # Build SoC.
    soc = BaseSoC(toolchain=args.toolchain)

    # Builder.
    builder = Builder(soc, csr_csv="csr.csv", bios_console="lite")
    builder.build(run=args.build)

    # Load Bistream.
    if args.load:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".svf"))

    # Flash Bitstream.
    if args.flash:
        prog = soc.platform.create_programmer(cable=args.cable)
        prog.flash(0, builder.get_bitstream_filename(mode="sram", ext=".svf"))

if __name__ == "__main__":
    main()
