#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *
from litex.gen import *
from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect import stream
from litex.soc.interconnect.stream import Converter, Gearbox

class sample_unpack128(LiteXModule):
    def __init__(self, enable_3Ch_mode=True
                 ):
        self.sink = AXIStreamInterface(128)
        self.source = AXIStreamInterface(128)
        self.ch_en = Signal(4)
        self.ch_en_reg = Signal(4)
        self.ch_en_changed = Signal()
        self.ch_en_changed_reset = Signal()
        self.global_ready = Signal(reset=0)

        self.intermediate_data = Signal(128)

        self.num_chans = Signal(2)
        # To avoid invalid values, these are the values:
        # 00: 1 channel active
        # 01: 2 channels active
        # 10: 3 channels active
        # 11: 4 channels active
        # We will determine ch_en pop count with parallel summation
        num_chans_var0 = Signal(2)
        num_chans_var1 = Signal(2)
        num_chans_var3 = Signal(3)
        # This will take 3 cycles to compute
        # FSM should be busy for longer than this
        self.sync += [
            # Add two pairs of adjacent bits
            num_chans_var0.eq(self.ch_en[0] + self.ch_en[1]),
            num_chans_var1.eq(self.ch_en[2] + self.ch_en[3]),
            # Add both pairs
            num_chans_var3.eq(num_chans_var0 + num_chans_var1),
            # Adjust the final value to fit in 2 bits
            self.num_chans.eq(num_chans_var3 - 1)
        ]

        self.conv1ch = ResetInserter()(Converter(128, 32))
        self.conv2ch = ResetInserter()(Converter(128, 64))
        self.conv3ch = ResetInserter()(Gearbox(128, 96))
        self.comb += [
            self.conv1ch.sink.data.eq(self.sink.data),
            self.conv2ch.sink.data.eq(self.sink.data),
             self.conv3ch.sink.data.eq(self.sink.data),
        ]

        # --------------------------------------------------
        # Multiplex width converters
        # --------------------------------------------------
        cases = dict()
        cases[0] = [
            # Connect converter to module sink
            self.conv1ch.sink.valid.eq(self.sink.valid),
            self.sink.ready.eq(self.conv1ch.sink.ready),
            # Keep in reset until global ready is set
            self.conv1ch.reset.eq(~self.global_ready),
            # Keep other converters in reset
            self.conv2ch.reset.eq(1),
             self.conv3ch.reset.eq(1),
            # Assign output from the converter
            self.intermediate_data[0:32].eq(self.conv1ch.source.data),
            self.intermediate_data[32:128].eq(0),
            # Assign control signals
            self.source.valid.eq(self.conv1ch.source.valid),
            self.conv1ch.source.ready.eq(self.source.ready),
        ]
        cases[1] = [
            # Connect converter to module sink
            self.conv2ch.sink.valid.eq(self.sink.valid),
            self.sink.ready.eq(self.conv2ch.sink.ready),
            # Keep other converters in reset
            self.conv1ch.reset.eq(1),
            self.conv2ch.reset.eq(~self.global_ready),
             self.conv3ch.reset.eq(1),
            # Assign output from the converter
            self.intermediate_data[0:64].eq(self.conv2ch.source.data),
            self.intermediate_data[64:128].eq(0),
            # Assign control signals
            self.source.valid.eq(self.conv2ch.source.valid),
            self.conv2ch.source.ready.eq(self.source.ready),
        ]
        cases[2] = [
            # Connect converter to module sink
             self.conv3ch.sink.valid.eq(self.sink.valid),
             self.sink.ready.eq(self.conv3ch.sink.ready),
            # Keep other converters in reset
            self.conv1ch.reset.eq(1),
            self.conv2ch.reset.eq(1),
             # self.conv3ch.reset.eq(~self.global_ready),
            # Assign output from the converter
             self.intermediate_data[0:96].eq(self.conv3ch.source.data),
             self.intermediate_data[96:128].eq(0),
            # Assign control signals
             self.source.valid.eq(self.conv3ch.source.valid),
             self.conv3ch.source.ready.eq(self.source.ready),
        ]
        if enable_3Ch_mode:
            cases[2].append(self.conv3ch.reset.eq(~self.global_ready))
        else:
            cases[2].append(self.conv3ch.reset.eq(1))
        # Directly connect input with output, no packing required
        # if all 4 channels are enabled
        cases[3] = [
            # Keep all converters in reset
            self.conv1ch.reset.eq(1),
            self.conv2ch.reset.eq(1),
             self.conv3ch.reset.eq(1),
            # Full bypass, ignore all other logic, except for the muxing
            self.intermediate_data.eq(self.sink.data),
            self.source.valid.eq(self.sink.valid & self.global_ready),
            self.sink.ready.eq(self.source.ready & self.global_ready),
        ]

        self.comb += Case(self.num_chans, cases)

        # --------------------------------------------------
        # Figure out output assignments
        # --------------------------------------------------
        # Output muxing signals
        # ch. 1 can either be at the bottom of intermediate or
        # not present at all
        # 0 - ch is not present
        # 1 - ch is at 0:31
        self.ch1_mux_pointer = Signal(1)
        # ch. 2 can be in two bottom places or not present
        # 0 - ch is not present
        # 1 - ch is at 0:31
        # 2 - ch is at 32:63
        self.ch2_mux_pointer = Signal(2)
        # ch. 3 can be in three bottom places or not present
        # 0 - ch is not present
        # 1 - ch is at 0:31
        # 2 - ch is at 32:63
        # 3 - ch is at 64:95
        self.ch3_mux_pointer = Signal(2)
        # ch. 4 can be in any of the 4 positions or not present
        # 0 - ch is not present
        # 1 - ch is at 0:31
        # 2 - ch is at 32:63
        # 3 - ch is at 64:95
        # 4 - ch is at 96:127
        self.ch4_mux_pointer = Signal(3)
        # number of active channels before this one
        self.active_chans_before = Signal(2)

        # Track if ch_en changes
        self.sync += [
            self.ch_en_reg.eq(self.ch_en),
            If(self.ch_en_changed_reset, [
                self.ch_en_changed.eq(0)
            ]).Elif(self.ch_en_reg != self.ch_en, [
                self.ch_en_changed.eq(1)
            ])
        ]

        # FSM
        self.submodules.fsm = fsm = FSM(reset_state="reset")

        # Add states
        # Always reevaluate output muxing after reset
        fsm.act("reset", [
            NextValue(self.active_chans_before, 0),
            NextValue(self.global_ready, 0),
            NextState("CH1"),
            # Make sure ch_en reset is not on
            NextValue(self.ch_en_changed_reset, 0)]
                )

        # If not in reset, evaluate muxing after ch_en changes
        fsm.act("wait_change", [
            # Make sure ch_en reset is not on
            NextValue(self.ch_en_changed_reset, 0),
            If(self.ch_en_changed, [
                NextValue(self.active_chans_before, 0),
                NextValue(self.global_ready, 0),
                NextState("CH1"),
            ])
        ]
                )

        fsm.act("CH1", [
            # Handle CH1 value
            If(self.ch_en[0], [
                # channel 1 is present
                NextValue(self.ch1_mux_pointer, 1),
                # active_chans_before should only ever be 0 at this moment
                # and thus can only become 1, but use full logic for consistency
                NextValue(self.active_chans_before, self.active_chans_before + 1),
            ]).Else([
                # channel 1 is not present
                NextValue(self.ch1_mux_pointer, 0),
            ]),
            NextState("CH2")
        ]
                )

        fsm.act("CH2", [
            # Handle CH2 value
            If(self.ch_en[1], [
                NextValue(self.ch2_mux_pointer, self.active_chans_before + 1),
                NextValue(self.active_chans_before, self.active_chans_before + 1),
            ]).Else([
                NextValue(self.ch2_mux_pointer, 0),
            ]),
            NextState("CH3")
        ]
                )

        fsm.act("CH3", [
            # Handle CH3 value
            If(self.ch_en[2], [
                NextValue(self.ch3_mux_pointer, self.active_chans_before + 1),
                NextValue(self.active_chans_before, self.active_chans_before + 1),
            ]).Else([
                NextValue(self.ch3_mux_pointer, 0),
            ]),
            NextState("CH4")
        ]
                )

        fsm.act("CH4", [
            # Handle CH4 value
            If(self.ch_en[3], [
                NextValue(self.ch4_mux_pointer, self.active_chans_before + 1),
                # Reset active_chans_before value before finishing just in case
                NextValue(self.active_chans_before, 0),
            ]).Else([
                NextValue(self.ch4_mux_pointer, 0)
            ]),
            NextValue(self.global_ready, 1),
            NextState("wait_change"),
            # Reset the sticky ch_en_changed signal
            NextValue(self.ch_en_changed_reset, 1)
        ]

                )

        # --------------------------------------------------
        # Do the actual muxing
        # --------------------------------------------------
        data_slices = Array([
            self.intermediate_data[0:32],
            self.intermediate_data[32:64],
            self.intermediate_data[64:96],
            self.intermediate_data[96:128]
        ])
        self.comb += [
            If(self.ch1_mux_pointer == 0, [
                self.source.data[0:32].eq(0),
            ]).Else([
                self.source.data[0:32].eq(self.intermediate_data[0:32]),
            ]),

            If(self.ch2_mux_pointer == 0, [
                self.source.data[32:64].eq(0),
            ]).Else([
                self.source.data[32:64].eq(data_slices[self.ch2_mux_pointer - 1]),
            ]),

            If(self.ch3_mux_pointer == 0, [
                self.source.data[64:96].eq(0),
            ]).Else([
                 self.source.data[64:96].eq(data_slices[self.ch3_mux_pointer - 1]),
            ]),

            If(self.ch4_mux_pointer == 0, [
                self.source.data[96:128].eq(0),
            ]).Else([
                self.source.data[96:128].eq(data_slices[self.ch4_mux_pointer - 1]),
            ])
        ]
