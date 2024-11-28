#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os

from migen import *

from litex.gen import *

from litex.build.vhd2v_converter import VHD2VConverter

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

        self.busy_delay_params = dict()
        self.busy_delay_params.update(
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

    def do_finalize(self):
        self.busy_delay = VHD2VConverter(self.platform,
            top_entity    = "busy_delay",
            build_dir     = os.path.join(os.path.abspath(self.platform.output_dir), "vhd2v"),
            work_package  = "work",
            force_convert = LiteXContext.platform.vhd2v_force,
            params        = self.busy_delay_params,
            add_instance  = True,
        )
        self.busy_delay.add_source("gateware/hdl/general/busy_delay.vhd")
