#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

# Busy Delay ---------------------------------------------------------------------------------------

class BusyDelay(LiteXModule):
    def __init__(self, platform, cd="sys",
        clock_period       = 10,
        delay_time         = 100,
        ):

        self.busy_in  = Signal()
        self.busy_out = Signal()

        # # #

        # busy_delay instance.
        # -------------------------

        self.specials += Instance("busy_delay",
            # Parameters
            p_clock_period = clock_period,
            p_delay_time   = delay_time,

            # Clk/Reset
            i_clk          = ClockSignal(cd),
            i_reset_n      = ~ResetSignal(cd),

            # busy IN/OUT
            i_busy_in      = self.busy_in,
            o_busy_out     = self.busy_out,
        )

        self.add_sources(platform)

    def add_sources(self, platform):
        platform.add_source("gateware/hdl/general/busy_delay.vhd")
