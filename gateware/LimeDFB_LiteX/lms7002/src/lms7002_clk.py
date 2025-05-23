#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.build.io import DDROutput

# LMS7002 CLK --------------------------------------------------------------------------------------

class LMS7002CLK(LiteXModule):
    def __init__(self, platform, pads=None, pllcfg_manager=None,
        drct_c0_ndly   = 1,
        drct_c1_ndly   = 8,
        drct_c2_ndly   = 1,
        drct_c3_ndly   = 8,
        with_max10_pll = True,
        ):
        # Configuration
        self.sel            = Signal() # 0 - fclk1 control, 1 - fclk2 control
        self.cflag          = Signal()
        self.direction      = Signal()
        self.loadn          = Signal()
        self.move           = Signal()

        self.rx_clk         = Signal()
        self.tx_clk         = Signal()

        # mini V1 only
        self.clk_ena        = Signal(4)
        self.drct_clk_en    = Signal(4)
        self.pll_locked     = Signal()
        self.smpl_cmp_en    = Signal()
        self.smpl_cmp_done  = Signal()
        self.smpl_cmp_error = Signal()
        self.smpl_cmp_cnt   = Signal(16)

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
            self.comb += [
                pads.FCLK1.eq(                   inst1_q),
                pads.FCLK2.eq(                   inst2_q),
            ]

            if with_max10_pll:
                assert pllcfg_manager is not None
                from gateware.max10_pll_top.max10_pll_top import MAX10PLLTop

                self.max10_pll = MAX10PLLTop(platform, pads, pllcfg_manager,
                    drct_c0_ndly = drct_c0_ndly,
                    drct_c1_ndly = drct_c1_ndly,
                    drct_c2_ndly = drct_c2_ndly,
                    drct_c3_ndly = drct_c3_ndly,
                )

                self.comb += [
                    self.max10_pll.clk_ena.eq(       self.clk_ena),
                    self.max10_pll.drct_clk_en.eq(   self.drct_clk_en),
                    self.pll_locked.eq(              self.max10_pll.pll_locked),
                    self.smpl_cmp_en.eq(             self.max10_pll.smpl_cmp_en),
                    self.max10_pll.smpl_cmp_done.eq( self.smpl_cmp_done),
                    self.max10_pll.smpl_cmp_error.eq(self.smpl_cmp_error),
                    self.smpl_cmp_cnt.eq(            self.max10_pll.smpl_cmp_cnt),

                    c0_global.eq(                    self.max10_pll.c0_global),
                    self.tx_clk.eq(                  self.max10_pll.tx_clk),
                    c2_global.eq(                    self.max10_pll.c2_global),
                    self.rx_clk.eq(                  self.max10_pll.rx_clk),
                ]
            else:
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
                ]

        else:
            from gateware.lms7002_clk import ClkCfgRegs
            from gateware.lms7002_clk import XilinxLmsMMCM
            from gateware.lms7002_clk import ClkMux
            from gateware.lms7002_clk import ClkDlyFxd

            # Clocking control registers
            self.CLK_CTRL = ClkCfgRegs()

            # TX clk
            # Xilinx MMCM is used to support configurable interface frequencies >5NHz
            # Muxed and delayed clock version is used for interface frequencies <5MHz

            # Global TX CLock
            self.cd_txclk_global = ClockDomain()
            self.comb += self.cd_txclk_global.clk.eq(pads.MCLK1)

            # TX PLL.
            self.cd_txpll_clk_c0 = ClockDomain()
            self.cd_txpll_clk_c1 = ClockDomain()

            self.PLL0_TX = XilinxLmsMMCM(platform, speedgrade=-2, max_freq=122.88e6,
                mclk      = self.cd_txclk_global.clk,
                fclk      = self.cd_txpll_clk_c0.clk,
                logic_cd  = self.cd_txpll_clk_c1)

            #TX CLK C0 mux
            self.cd_txclk_c0_muxed = ClockDomain()
            self.txclk_mux         = ClkMux(
                i0  = self.cd_txpll_clk_c0.clk,
                i1  = self.cd_txclk_global.clk,
                o   = self.cd_txclk_c0_muxed.clk,
                sel = self.CLK_CTRL.DRCT_TXCLK_EN.storage)

            #TX CLK C1 delay
            self.cd_txclk_c1_dly = ClockDomain()
            self.txclk_c1_dlly   = ClkDlyFxd(i=self.cd_txclk_global.clk, o=self.cd_txclk_c1_dly.clk)

            #TX CLK C1 mux
            self.cd_txclk  = ClockDomain()
            self.txclk_mux = ClkMux(
                i0  = self.cd_txpll_clk_c1.clk,
                i1  = self.cd_txclk_c1_dly.clk,
                o   = self.cd_txclk.clk,
                sel = self.CLK_CTRL.DRCT_TXCLK_EN.storage)

            # Create clock groups (false paths) between sys clk and all clocks from TX interface tree
            platform.add_false_path_constraints(
                LiteXContext.top.crg.cd_sys.clk,
                self.cd_txclk_global.clk,
                self.cd_txpll_clk_c0.clk,
                self.cd_txpll_clk_c1.clk,
                self.cd_txclk_c0_muxed.clk,
                self.cd_txclk_c1_dly.clk,
                self.cd_txclk.clk,
            )

            self.comb += [
                self.tx_clk.eq(self.cd_txclk.clk),
                pads.FCLK1.eq(self.cd_txclk_c0_muxed.clk),
            ]

            # RX clk
            # Xilinx MMCM is used to support configurable interface frequencies >5NHz
            # Muxed and delayed clock version is used for interface frequencies <5MHz

            # Global RX CLock
            self.cd_rxclk_global = ClockDomain()
            self.comb += self.cd_rxclk_global.clk.eq(pads.MCLK2)

            # RX PLL.
            self.cd_rxpll_clk_c0 = ClockDomain()
            self.cd_rxpll_clk_c1 = ClockDomain()
            self.PLL1_RX         = XilinxLmsMMCM(platform, speedgrade=-2, max_freq=122.88e6,
                mclk     = self.cd_rxclk_global.clk,
                fclk     = self.cd_rxpll_clk_c0.clk,
                logic_cd = self.cd_rxpll_clk_c1)

            #RX CLK C0 mux
            self.cd_rxclk_c0_muxed = ClockDomain()
            self.rxclk_mux         = ClkMux(
                i0  = self.cd_rxpll_clk_c0.clk,
                i1  = self.cd_rxclk_global.clk,
                o   = self.cd_rxclk_c0_muxed.clk,
                sel = self.CLK_CTRL.DRCT_RXCLK_EN.storage)

            #RX CLK C1 delay
            self.cd_rxclk_c1_dly = ClockDomain()
            self.rxclk_c1_dlly   = ClkDlyFxd(i=self.cd_rxclk_global.clk, o=self.cd_rxclk_c1_dly.clk)

            #RX CLK C1 mux
            self.cd_rxclk  = ClockDomain()
            self.rxclk_mux = ClkMux(
                i0  = self.cd_rxpll_clk_c1.clk,
                i1  = self.cd_rxclk_c1_dly.clk,
                o   = self.cd_rxclk.clk,
                sel = self.CLK_CTRL.DRCT_RXCLK_EN.storage,
            )

            # Create clock groups (false paths) between sys clk and all clocks from RX interface tree
            platform.add_false_path_constraints(
                LiteXContext.top.crg.cd_sys.clk,
                self.cd_rxclk_global.clk,
                self.cd_rxpll_clk_c0.clk,
                self.cd_rxpll_clk_c1.clk,
                self.cd_rxclk_c0_muxed.clk,
                self.cd_rxclk_c1_dly.clk,
                self.cd_rxclk.clk,
            )

            self.comb += [
                self.rx_clk.eq(self.cd_rxclk.clk),
                pads.FCLK2.eq(self.cd_rxclk_c0_muxed.clk),
            ]
