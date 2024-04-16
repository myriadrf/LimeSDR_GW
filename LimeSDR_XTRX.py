#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Build/Use ----------------------------------------------------------------------------------------
# Build/Load bitstream:
# ./LimeSDR_XTRX.py --integrated-main-ram-size 0x8000 --build --load --uart-name=jtag_uart --cpu-type=vexriscv_smp --with-rvc  --with-privileged-debug --hardware-breakpoints 4
#
# Build firmware:
# cd firmware && make clean all && cd ../
#
# Load CPU firmware:
# litex_term jtag --jtag-config=openocd_xc7_ft2232.cfg --kernel firmware/demo.bin

import os

from migen import *

from litex.gen import *

import LimeSDR_XTRX_platform as limesdr_xtrx

from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.soc.cores.led import LedChaser
from litex.soc.cores.clock import *

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_pcie=False):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        self.pll_clk_in = Signal()
        self.vctcxo_pads = platform.request("vctcxo")

        self.comb += self.pll_clk_in.eq(self.vctcxo_pads.FPGA_CLK)
        self.comb += self.vctcxo_pads.EN_TCXO.eq(1)
        self.comb += self.vctcxo_pads.EXT_CLK.eq(0)

        # # #

        if with_pcie:
            assert sys_clk_freq == int(125e6)
            self.comb += [
                self.cd_sys.clk.eq(ClockSignal("pcie")),
                self.cd_sys.rst.eq(ResetSignal("pcie")),
            ]
        else:
            self.pll = pll = S7PLL(speedgrade=-2)
            self.comb += pll.reset.eq(self.rst)
            pll.register_clkin(self.pll_clk_in, 26e6)
            pll.create_clkout(self.cd_sys, sys_clk_freq)

# BaseSoC -----------------------------------------------------------------------------------------


# Create a led blinker module
class Blink(Module):
    def __init__(self, led):
        counter = Signal(26)
        # combinatorial assignment
        self.comb += led.eq(counter[25])

        # synchronous assignement
        self.sync += counter.eq(counter + 1)


class VCTCXO(Module):
    def __init__(self, pads):
        pads.EN_TCXO.eq(1)
        pads.EXT_CLK.eq(0)


class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=125e6, with_pcie=False, with_led_chaser=True, **kwargs):
        platform = limesdr_xtrx.Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = CRG(platform, sys_clk_freq, with_pcie)

        # SoCCore ----------------------------------------------------------------------------------
        if kwargs["uart_name"] != "jtag_uart":
            kwargs["uart_name"]     = "crossover"
            kwargs["with_jtagbone"] = True
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on LimeSDR-XTRX", **kwargs)

        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x2"),
                data_width = 64,
                bar0_size  = 0x20000)
            self.add_pcie(phy=self.pcie_phy, ndmas=1)

            # ICAP (For FPGA reload over PCIe).
            from litex.soc.cores.icap import ICAP
            self.icap = ICAP()
            self.icap.add_reload()
            self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

            # Flash (For SPIFlash update over PCIe).
            from litex.soc.cores.gpio import GPIOOut
            from litex.soc.cores.spi_flash import S7SPIFlash
            self.flash_cs_n = GPIOOut(platform.request("FPGA_CFG_CS"))
            self.flash      = S7SPIFlash(platform.request("flash"), sys_clk_freq, 25e6)


        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("FPGA_LED1"),
                sys_clk_freq = sys_clk_freq)

        self.blinker = Blink(
            led=platform.request_all("FPGA_LED2")
        )

        #self.comb += platform.request("vctcxo").EN_TCXO.eq(1)
        self.platform.add_platform_command(
            "create_clock -name vctcxo_FPGA_CLK -period {} [get_ports vctcxo_FPGA_CLK]".format(1e9 / 26e6))

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=limesdr_xtrx.Platform, description="LiteX SoC on LimeSDR-XTRX.")
    parser.add_target_argument("--flash",           action="store_true",       help="Flash bitstream.")
    parser.add_target_argument("--sys-clk-freq",    default=125e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-pcie",       action="store_true",       help="Enable PCIe support.")
    parser.add_target_argument("--driver",          action="store_true",       help="Generate PCIe driver.")
    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq = args.sys_clk_freq,
        with_pcie    = args.with_pcie,
        **parser.soc_argdict
    )
    builder  = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
