#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

# Ref: https://www.intel.com/content/www/us/en/docs/programmable/683865/current/fpga-configuration-overview.html#sss1397549479122

import os

from migen import *

from litex.gen import *

from litex.soc.interconnect import wishbone

# Max10 Dual Configuration -------------------------------------------------------------------------

class Max10DualCfg(LiteXModule):
    def __init__(self, platform):

        self.bus      = wishbone.Interface(data_width=32, adr_width=3)
        self.platform = platform

        # # #

        # Signals.
        # --------

        # Avalong Interface
        avmm_read  = Signal()
        avmm_write = Signal()

        # max10_dual_cfg Instance.
        # ------------------------
        self.specials += Instance("max10_dual_cfg",
            # Clk/Reset.
            i_clk                 = ClockSignal("sys"),
            i_nreset              = ~ResetSignal("sys"),

            # Avalon MM
            i_avmm_rcv_address   = self.bus.adr,
            i_avmm_rcv_read      = ~self.bus.we & self.bus.stb,
            o_avmm_rcv_readdata  = self.bus.dat_r,
            i_avmm_rcv_write     = self.bus.we & self.bus.stb,
            i_avmm_rcv_writedata = self.bus.dat_w,
        )

        # Logic.
        # ------

        # Connect Avalon Interface to Wishbone.
        self.comb += [
            self.bus.ack.eq(avmm_write | avmm_read),
            avmm_write.eq(  self.bus.we & self.bus.stb & ~self.bus.ack),
            avmm_read.eq(  ~self.bus.we & self.bus.stb & ~self.bus.ack),
        ]

        self.add_sources(platform)

    def add_sources(self, platform):
        base_path = "gateware/max10_dual_cfg/synthesis"
        files = [
            "max10_dual_cfg.v",
            "submodules/altera_dual_boot.v",
            "submodules/rtl/alt_dual_boot.v",
            "submodules/rtl/alt_dual_boot_avmm.v",
        ]

        for file in files:
            platform.add_source(os.path.join(base_path, file))

