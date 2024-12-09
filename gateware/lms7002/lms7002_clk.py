#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.build.io import DDROutput

# LMS7002 CLK --------------------------------------------------------------------------------------

class LMS7002CLK(LiteXModule):
    def __init__(self, platform, pads=None, drct_c0_ndly=1, drct_c2_ndly=1):
        # Configuration
        self.sel       = Signal() # 0 - fclk1 control, 1 - fclk2 control
        self.cflag     = Signal()
        self.direction = Signal()
        self.loadn     = Signal()
        self.move      = Signal()

        self.rx_clk    = Signal()
        self.tx_clk    = Signal()

        # mini V1 only
        self.clk_ena     = Signal(3)
        self.drct_clk_en = Signal(4)

        # # #

        # Signals.
        # --------
        inst1_q = Signal()
        inst2_q = Signal()

        inst3_loadn      = Signal()
        inst3_move       = Signal()
        inst3_direction  = Signal()
        inst3_cflag      = Signal()

        inst4_loadn      = Signal()
        inst4_move       = Signal()
        inst4_direction  = Signal()
        inst4_cflag      = Signal()

        inst3_loadn.attr.add("keep")
        inst3_move.attr.add("keep")
        inst3_direction.attr.add("keep")
        inst3_cflag.attr.add("keep")

        inst4_loadn.attr.add("keep")
        inst4_move.attr.add("keep")
        inst4_direction.attr.add("keep")
        inst4_cflag.attr.add("keep")

        # Control logic.
        # --------------
        self.comb += [
            If(self.sel,
                inst3_loadn.eq(1),
                inst3_move.eq( 0),
                inst4_loadn.eq(self.loadn),
                inst4_move.eq( self.move),
                self.cflag.eq( inst4_cflag),
            ).Else(
                inst3_loadn.eq(self.loadn),
                inst3_move.eq( self.move),
                inst4_loadn.eq(1),
                inst4_move.eq( 0),
                self.cflag.eq( inst3_cflag),
            ),
            inst3_direction.eq(self.direction),
            inst4_direction.eq(self.direction),
        ]

        if platform.name in ["limesdr_mini_v1", "limesdr_mini_v2"]:
            c0_global         = Signal()
            c2_global         = Signal()

            self.specials += [
                # Forwarded clock fclk1.
                # ----------------------
                DDROutput(
                    clk = c0_global,
                    i1  = {True:0, False:1}[platform.name == "limesdr_mini_v2"],
                    i2  = {True:1, False:0}[platform.name == "limesdr_mini_v2"],
                    o   = inst1_q
                ),

                # Forwarded clock fclk2.
                # ----------------------
                DDROutput(
                    clk = c2_global,
                    i1  = {True:0, False:1}[platform.name == "limesdr_mini_v2"],
                    i2  = {True:1, False:0}[platform.name == "limesdr_mini_v2"],
                    o   = inst2_q
                )
            ]
            self.comb += [
                self.tx_clk.eq(pads.MCLK1),
                self.rx_clk.eq(pads.MCLK2),
            ]
        if platform.name in ["limesdr_mini_v2"]:
            self.comb += [
                c0_global.eq(pads.MCLK1),
                c2_global.eq(pads.MCLK2),
            ]

            self.specials += [
                Instance("DELAYF",
                    p_DEL_VALUE = 1,
                    p_DEL_MODE  = "USER_DEFINED",
                    i_A         = inst1_q,
                    i_LOADN     = inst3_loadn,
                    i_MOVE      = inst3_move,
                    i_DIRECTION = inst3_direction,
                    o_Z         = pads.FCLK1,
                    o_CFLAG     = inst3_cflag,
                ),
                Instance("DELAYF",
                    p_DEL_VALUE = 1,
                    p_DEL_MODE  = "USER_DEFINED",
                    i_A         = inst2_q,
                    i_LOADN     = inst4_loadn,
                    i_MOVE      = inst4_move,
                    i_DIRECTION = inst4_direction,
                    o_Z         = pads.FCLK2,
                    o_CFLAG     = inst4_cflag,
                ),
            ]
        elif platform.name in ["limesdr_mini_v1"]:
            inst3_clk         = Signal(3)
            # TX.
            # ---
            drct_c0_dly_chain = Signal(drct_c0_ndly)
            c0_mux            = Signal()

            for i in range(drct_c0_ndly):
                self.specials += Instance("lcell",
                    i_in  = {True:pads.MCLK2, False:drct_c0_dly_chain[i-1]}[i==0],
                    o_out = drct_c0_dly_chain[i],
                )
            self.specials += [
                Instance("fiftyfivenm_clkctrl",
                    p_clock_type        = "Global Clock",
                    p_ena_register_mode = "falling edge",
                    p_lpm_type          = "fiftyfivenm_clkctrl",

                    i_inclk             = c0_mux,
                    i_clkselect         = Constant(0, 2),
                    i_ena               = self.clk_ena[0],
                    o_outclk            = c0_global,
                    # io_devclrn          = Constant(1, 1),
                    # io_devpor           = Constant(1, 1),
                ),
            ]

            self.comb += [
                If(self.drct_clk_en[0],
                    c0_mux.eq(drct_c0_dly_chain[drct_c0_ndly-1]),
                ).Else(
                    c0_mux.eq(pads.MCLK2)
                ),
                pads.FCLK1.eq(inst1_q),
            ]

            # RX.
            # ---
            drct_c2_dly_chain = Signal(drct_c2_ndly)
            c2_mux            = Signal()

            for i in range(drct_c2_ndly):
                self.specials += Instance("lcell",
                    i_in  = {True:pads.MCLK2, False:drct_c2_dly_chain[i-1]}[i==0],
                    o_out = drct_c2_dly_chain[i],
                )
            self.specials += [
                Instance("fiftyfivenm_clkctrl",
                    p_clock_type        = "Global Clock",
                    p_ena_register_mode = "falling edge",
                    p_lpm_type          = "fiftyfivenm_clkctrl",

                    i_inclk             = c2_mux,
                    i_clkselect         = Constant(0, 2),
                    i_ena               = self.clk_ena[2],
                    o_outclk            = c2_global,
                    # io_devclrn          = Constant(1, 1),
                    # io_devpor           = Constant(1, 1),
                ),
            ]
            self.comb += [
                If(self.drct_clk_en[2],
                    c2_mux.eq(drct_c2_dly_chain[drct_c2_ndly-1]),
                ).Else(
                    c2_mux.eq(pads.MCLK2),
                ),
                pads.FCLK2.eq(inst2_q),
            ]

        else:
            from gateware.lms7002_clk import ClkCfgRegs
            from gateware.lms7002_clk import XilinxLmsMMCM

            # Clocking control registers
            self.CLK_CTRL = ClkCfgRegs()
            # TX PLL.
            self.txclk    = ClockDomain()
            self.PLL0_TX  = XilinxLmsMMCM(platform, speedgrade=-2, max_freq=122.88e6,
                mclk     = pads.MCLK1,
                fclk     = pads.FCLK1,
                logic_cd = self.txclk)
            self.comb += self.tx_clk.eq(self.txclk.clk)
            # RX PLL.
            self.rxclk    = ClockDomain()
            self.PLL1_RX  = XilinxLmsMMCM(platform, speedgrade=-2, max_freq=122.88e6,
                mclk     = pads.MCLK2,
                fclk     = pads.MCLK2,
                logic_cd = self.rxclk)
            self.comb += self.rx_clk.eq(self.rxclk.clk)
