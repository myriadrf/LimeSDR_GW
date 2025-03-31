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

        self.sink      = AXIStreamInterface(sink_width, sink_width//8, clock_domain=sink_clk_domain)
        self.source    = AXIStreamInterface(source_width, source_width//8, clock_domain=source_clk_domain)

        self.reset = Signal()
        buffer_rst = Signal()

        self.buffer_rst = buffer_rst

        self.debug_wr_cnt = Signal(16)

        # Input data buffer (128 bit)
        self.input_buff = ResetInserter()(ClockDomainsRenamer(source_clk_domain)(stream.SyncFIFO([("data", sink_width), ("keep", sink_width//8)], depth=512, buffered=True)))
        self.output_buff = ResetInserter()(ClockDomainsRenamer(source_clk_domain)(
            stream.SyncFIFO([("data", source_width), ("keep", source_width // 8)], depth=16, buffered=True)))

        self.comb += [
            self.input_buff.reset.eq(self.reset),
            self.input_buff.sink.valid.eq(self.sink.valid),
            self.sink.ready.eq(self.input_buff.sink.ready),
            self.input_buff.sink.data.eq(self.sink.data),
            self.input_buff.sink.keep.eq(self.sink.keep),

            self.output_buff.reset.eq(self.reset),
            self.source.valid.eq(self.output_buff.source.valid),
            self.output_buff.source.ready.eq(self.source.ready),
            self.source.data.eq(self.output_buff.source.data),
            self.source.keep.eq(self.output_buff.source.keep),
            self.source.last.eq(self.output_buff.source.last),
            ]

        # Instantiate the FFT wrapper.
        self.fft_wrap = Instance("fft_wrap",
                                 i_CLK              = ClockSignal(source_clk_domain),
                                 i_RESET_N          = ~self.reset,
                                 i_S_AXIS_TVALID    = self.input_buff.source.valid,
                                 i_S_AXIS_TDATA     = self.input_buff.source.data,
                                 o_S_AXIS_TREADY    = self.input_buff.source.ready,
                                 i_S_AXIS_TLAST     = self.input_buff.source.last,
                                 i_S_AXIS_TKEEP     = self.input_buff.source.keep,
                                 o_M_AXIS_TDATA     = self.output_buff.sink.data,
                                 o_M_AXIS_TVALID    = self.output_buff.sink.valid,
                                 i_M_AXIS_TREADY    = self.output_buff.sink.ready,
                                 o_M_AXIS_TLAST     = self.output_buff.sink.last,
                                 o_M_AXIS_TKEEP     = self.output_buff.sink.keep,
                                 o_buff_rst         = buffer_rst,
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