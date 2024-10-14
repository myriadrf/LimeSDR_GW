#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Build/Use:
# ./limesdr_mini_v2.py --csr-csv=csr.csv --build --load
# litex_server --jtag --jtag-config=openocd_limesdr_mini_v2.cfg
# litex_term crossover

# loading a demo
# ./limesdr_mini_v2.py --integrated-main-ram-size 0x8000 --load --build --uart-name=jtag_uart
# litex_bare_metal_demo --build-path build/limesdr_mini_v2
# litex_term jtag --jtag-config=openocd_limesdr_mini_v2.cfg --kernel demo.bin

from migen import *

from litex.gen import *

import limesdr_mini_v2_platform as limesdr_mini_v2

from litex.soc.cores.clock import *
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream

from litex.soc.cores.led import LedChaser
from litex.soc.cores.bitbang import I2CMaster
from litex.soc.cores.usb_fifo import FT245PHYSynchronous

from litescope import LiteScopeAnalyzer

from gateware.lms7_trx_top import LMS7TRXTopWrapper

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        self.cd_usb = ClockDomain()

        # # #

        # Clk.
        lmk_clk = platform.request("LMK_CLK")

        self.comb += self.cd_sys.clk.eq(lmk_clk)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=80e6, toolchain="diamond",
        with_usb_fifo   = True, with_usb_fifo_loopback=False,
        with_led_chaser = True,
        **kwargs):
        platform = limesdr_mini_v2.Platform(toolchain=toolchain)

        # SoCCore ----------------------------------------------------------------------------------
        if kwargs["uart_name"] != "jtag_uart":
            kwargs["uart_name"]     = "crossover"
            kwargs["with_jtagbone"] = True
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on LimeSDR-Mini-V2", **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # TOP --------------------------------------------------------------------------------------
        self.lms7_trx_top = LMS7TRXTopWrapper(self.platform)

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=limesdr_mini_v2.Platform, description="LiteX SoC on LimeSDR-Mini-V2.")
    parser.add_target_argument("--sys-clk-freq", default=40e6, type=float, help="System clock frequency.")
    args = parser.parse_args()
    args.no_serial = True
    args.cpu_type  = None
    args.toolchain = "diamond"

    soc = BaseSoC(
        sys_clk_freq = args.sys_clk_freq,
        toolchain    = args.toolchain,
        **parser.soc_argdict
    )
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram", ext=".svf")) # FIXME

if __name__ == "__main__":
    main()
