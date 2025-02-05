#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from gateware.common import add_vhd2v_converter

# Busy Delay ---------------------------------------------------------------------------------------

class BusyDelay(LiteXModule):
    def __init__(self, platform, cd="sys",
        clock_period       = 10,
        delay_time         = 100,
        ):

        self.platform = platform

        self.busy_in  = Signal()
        self.busy_out = Signal()

        # # #

        # busy_delay instance.
        # -------------------------

        self.busy_delay = Instance("busy_delay",
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

        files                = ["gateware/LimeDFB_LiteX/general/busy_delay.vhd"]
        self.busy_delay_conv = add_vhd2v_converter(self.platform, self.busy_delay, files)
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.busy_delay)
