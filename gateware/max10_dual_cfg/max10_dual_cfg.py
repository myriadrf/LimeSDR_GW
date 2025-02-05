#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
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

        # Avalon Interface
        self.avmm_addr  = avmm_addr  = Signal(3)
        self.avmm_read  = avmm_read  = Signal()
        self.avmm_rdata = avmm_rdata = Signal(32)
        self.avmm_write = avmm_write = Signal()
        self.avmm_wdata = avmm_wdata = Signal(32)

        # max10_dual_cfg Instance.
        # ------------------------
        self.specials += Instance("max10_dual_cfg",
            # Clk/Reset.
            i_clk                 = ClockSignal("sys"),
            i_nreset              = ~ResetSignal("sys"),

            # Avalon MM
            i_avmm_rcv_address   = avmm_addr,
            i_avmm_rcv_read      = avmm_read,
            o_avmm_rcv_readdata  = avmm_rdata,
            i_avmm_rcv_write     = avmm_write,
            i_avmm_rcv_writedata = avmm_wdata,
        )

        # Logic.
        # ------

        # Connect Avalon Interface to Wishbone.
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.bus.stb & self.bus.cyc,
                If(self.bus.we,
                    NextState("WRITE")
                ).Else(
                    NextState("READ-REQ")
                )
            ),
        )
        fsm.act("WRITE",
            avmm_write.eq(1),
            avmm_addr.eq(self.bus.adr),
            avmm_wdata.eq(self.bus.dat_w),
            self.bus.ack.eq(1),
            NextState("IDLE")
        )
        fsm.act("READ-REQ",
            avmm_read.eq(1),
            avmm_addr.eq(self.bus.adr),
            NextState("READ-DAT")
        )
        fsm.act("READ-DAT",
            self.bus.ack.eq(1),
            self.bus.dat_r.eq(avmm_rdata),
            NextState("IDLE")
        )

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

