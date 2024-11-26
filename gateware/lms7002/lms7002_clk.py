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
    def __init__(self, platform, pads=None):
        # Configuration
        self.sel       = Signal() # 0 - fclk1 control, 1 - fclk2 control
        self.cflag     = Signal()
        self.direction = Signal()
        self.loadn     = Signal()
        self.move      = Signal()

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

        self.specials += [
            # Forwarded clock fclk1.
            # ----------------------
            DDROutput(
                clk = pads.MCLK1,
                i1  = 0,
                i2  = 1,
                o   = inst1_q
            ),

            # Forwarded clock fclk2.
            # ----------------------
            DDROutput(
                clk = pads.MCLK2,
                i1  = 0,
                i2  = 1,
                o   = inst2_q
            )
        ]
        if platform.name in ["limesdr_mini_v2"]:
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
        else:
            print("LMS7002CLK Missing Delay!")
