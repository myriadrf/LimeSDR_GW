#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.build.io import DDROutput

# LMS7002 TXIQ -------------------------------------------------------------------------------------

class LMS7002TXIQ(LiteXModule):
    def __init__(self, platform, iq_width=12, pads=None):
        # Delay control
        self.data_loadn     = Signal()
        self.data_move      = Signal()
        self.data_direction = Signal()
        self.data_cflag     = Signal()
		# From internal logic
        self.tx_diq1_h      = Signal(iq_width + 1)
        self.tx_diq1_l      = Signal(iq_width + 1)

        # # #

        # Signals.
        # --------
        delay_z     = Signal(iq_width + 1)
        delay_cflag = Signal(iq_width + 1)

        for i in range(iq_width + 1):
            oddr_q = Signal()
            # ODDR component.
            # ----------------
            self.specials += [
                DDROutput(
                    clk = ClockSignal("lms_tx"),
                    i1  = self.tx_diq1_h[i],
                    i2  = self.tx_diq1_l[i],
                    o   = oddr_q,
                ),
            ]
            if platform.name in ["limesdr_mini_v2"]:
                self.specials += [
                    # Delay component.
                    # ----------------
                    Instance("DELAYF",
                        p_DEL_VALUE =  1,
                        p_DEL_MODE  = "USER_DEFINED",
                        i_A         = oddr_q,
                        i_LOADN     = self.data_loadn,
                        i_MOVE      = self.data_move,
                        i_DIRECTION = self.data_direction,
                        o_Z         = delay_z[i],
                        o_CFLAG     = delay_cflag[i]
                    ),
                ]
            else:
                print("LMS7002TXIQ Missing Delay!")
                self.comb += delay_z[i].eq(oddr_q)

        # Connect outputs.
        # ----------------
        self.comb += [
            pads.DIQ1_D.eq(       delay_z[0:iq_width]),
            pads.ENABLE_IQSEL1.eq(delay_z[iq_width]),
            self.data_cflag.eq(   Reduce("OR", delay_cflag)), # OR whole vector
        ]
