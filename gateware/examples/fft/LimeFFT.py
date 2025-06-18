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
        # To be compatible with LimeDFB modules, both sink and source should be 64bit wide.
        sink_width          =64,
        # Specifying clock sink/source clock domains as parameters allows them to be changed, if needed.
        sink_clk_domain     ="sys",
        source_width        =64,
        source_clk_domain   ="sys",
        ):

        # Create sink and source
        # Sink and Source interfaces have to be defined to allow for use of stream.pipeline()
        self.sink      = AXIStreamInterface(data_width=sink_width, keep_width=sink_width//8, clock_domain=sink_clk_domain)
        self.source    = AXIStreamInterface(data_width=source_width, keep_width=source_width//8, clock_domain=source_clk_domain)

        # Create reset signals
        self.reset = Signal()
        self.buffer_rst = Signal()

        # Input data buffer
        # ResetInserter()(module) creates an explicit reset signal for the module
        # ClockDomainsRenamer(clk_domain)(module) tells the module to use clk_domain instead of the default clock domain
        #
        self.input_buff = ResetInserter()(ClockDomainsRenamer(sink_clk_domain)(
            stream.SyncFIFO(layout=[("data", sink_width), ("keep", sink_width//8)], depth=512, buffered=True)))
        # Output buffer
        self.output_buff = ResetInserter()(ClockDomainsRenamer(source_clk_domain)(
            stream.SyncFIFO(layout=[("data", source_width), ("keep", source_width // 8)], depth=16, buffered=True)))

        # 'self.comb +=' adds an assignment or assignments to the list of combinatorial(async) assignments to be implemented by LiteX
        # 'self.sync +=' can be used to add synchronous assignments
        # 'self.sync.domain +=' can be used to add assignments with a specific clock domain, if 'domain' is a clock domain object
        # If only the name of the clock domain is available, but not the object, this construct can be used instead:
        # sync_domain = getattr(self.sync, "my_clock_name")
        # sync_domain +=
        #
        # signal_0.eq(signal_1) assigns the value of signal_1 to signal_0.
        # make sure that eq() statements used within the aforementioned constructs such as self.comb += or self.sync +=
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
        # 'of' parameter has to match the file/module name
        # 'i_port' indicates an input port
        # 'o_port' indicates an output port
        self.fft_wrap = Instance(of = "fft_wrap",
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
                                 o_buff_rst         = self.buffer_rst,
                                 )

        # Add FFT sources to the platform (verilog files can be added directly).
        platform.add_source("./gateware/examples/fft/fft.v")

        # Add FFT sources to the platform (VHDL files have to be converted to Verilog for some toolchains).
        fft_wrap_files = [
            "./gateware/examples/fft/fft_wrap.vhd",

        ]
        # Convert necessary source files
        self.fft_wrap_conv = add_vhd2v_converter(platform,
                                                 instance=self.fft_wrap,
                                                 files=fft_wrap_files,
                                                 # force_convert=False prevents conversion for toolchains that
                                                 # support VHDL
                                                 force_convert=False,
                                                 )

        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.fft_wrap)