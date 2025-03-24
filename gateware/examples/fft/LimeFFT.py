#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import math

from migen import *

from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect                import stream

from gateware.common import *

# LimeFFT ------------------------------------------------------------------------------------------

class LimeFFT(LiteXModule):
    def __init__(self, platform,
        # Configuration.
        sink_width          =64,
        sink_clk_domain     ="sys",
        source_width        =64,
        source_clk_domain   ="sys",
        ):

        self.sink      = AXIStreamInterface(sink_width,   clock_domain=sink_clk_domain)
        self.source    = AXIStreamInterface(source_width, clock_domain=source_clk_domain)

        self.reset = Signal()

        # Instantiate the FFT wrapper.
        self.fft_wrap = Instance("fft_wrap",
                                 i_CLK              = ClockSignal(source_clk_domain),
                                 i_RESET_N          = ~self.reset,
                                 i_S_AXIS_TVALID    = self.sink.valid,
                                 i_S_AXIS_TDATA     = self.sink.data,
                                 o_S_AXIS_TREADY    = self.sink.ready,
                                 i_S_AXIS_TLAST     = self.sink.last,
                                 i_S_AXIS_TKEEP     = self.sink.keep,
                                 o_M_AXIS_TDATA     = self.source.data,
                                 o_M_AXIS_TVALID    = self.source.valid,
                                 i_M_AXIS_TREADY    = self.source.ready,
                                 o_M_AXIS_TLAST     = self.source.last,
                                 o_M_AXIS_TKEEP     = self.source.keep,
                                 )

        # Add FFT sources to the platform.
        platform.add_source("./gateware/examples/fft/fft.v")
        # platform.add_source("./gateware/examples/fft/fft_wrap.vhd")

        fft_wrap_files = [
            "./gateware/examples/fft/fft_wrap.vhd",
            # "./gateware/examples/fft/fft.v"

        ]
        self.fft_wrap_conv = add_vhd2v_converter(platform,
                                                 instance=self.fft_wrap,
                                                 files=fft_wrap_files,
                                                 )

        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.fft_wrap)