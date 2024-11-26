#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.build.io import DDRInput

# LMS7002 RXIQ -------------------------------------------------------------------------------------

class LMS7002RXIQ(LiteXModule):
    def __init__(self, platform, iq_width=12, pads=None):
        # Delay control
        self.data_loadn     = Signal()
        self.data_move      = Signal()
        self.data_direction = Signal()
        self.data_cflag     = Signal()
        # Output ports to internal logic
        self.rx_diq2_h      = Signal(iq_width + 1)
        self.rx_diq2_l      = Signal(iq_width + 1)

        # # #

        # Signals.

        datain               = Signal(iq_width + 1)
        datain_h             = Signal(iq_width + 1)
        datain_l             = Signal(iq_width + 1)
        datain_reg_h         = Signal(iq_width + 1)
        datain_reg_l         = Signal(iq_width + 1)
        datain_reg_l_delayed = Signal(iq_width + 1)
        gen_delay_cflag      = Signal(iq_width + 1)

        for i in range(iq_width + 1):
            buf_datain   = Signal()
            delay_datain = Signal()

            if platform.name in ["limesdr_mini_v2"]:
                self.specials += [
                    # Input buffers.
                    # --------------
                    Instance("IB",
                        i_I = datain[i],
                        o_O = buf_datain,
                    ),
                    # Delay components.
                    # -----------------
                    Instance("DELAYF",
                        p_DEL_VALUE = 1,
                        p_DEL_MODE  = "USER_DEFINED",
                        i_A         = buf_datain,
                        i_LOADN     = self.data_loadn,
                        i_MOVE      = self.data_move,
                        i_DIRECTION = self.data_direction,
                        o_Z         = delay_datain,
                        o_CFLAG     = gen_delay_cflag[i],
                    )
                ]
            else:
                print("LMS7002RXIQ Missing Delay!")

            self.specials += [
                # IDDR components.
                # ----------------
                DDRInput(
                    clk = ClockSignal("lms_rx"),
                    i   = delay_datain,
                    o1  = datain_h[i],
                    o2  = datain_l[i],
                ),
            ]

            # Logic.
            # ------
            self.comb += datain.eq(Cat(pads.DIQ2_D, pads.ENABLE_IQSEL2))

            # Internal registers
            # Resync both on rising edge
            self.sync.lms_rx += [
                self.rx_diq2_h.eq(datain_h),
                datain_reg_l.eq(  datain_l),
                #- We need to delay data captured on falling edge, in order to allign samples
                self.rx_diq2_l.eq(     datain_reg_l),
            ]

            self.comb += self.data_cflag.eq(Reduce("OR", gen_delay_cflag)) # OR all vector bits
