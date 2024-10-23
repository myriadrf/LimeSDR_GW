#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import LiteXModule

from litex.soc.interconnect.csr import *

from gateware.common import FIFOInterface

# FIFO Control To CSR ------------------------------------------------------------------------------

class FIFOCtrlToCSR(LiteXModule):
    def __init__(self, EP02_rwidth=8, EP82_wwidth=8):
        self.ctrl_fifo   = FIFOInterface(EP02_rwidth, EP82_wwidth)
        self.fifo_reset  = Signal()

        # Write FIFO.
        self._fifo_wdata = CSRStorage(EP82_wwidth, description="FIFO Write Register.")

        # Read FIFO.
        self._fifo_rdata = CSRStatus(EP02_rwidth, description="FIFO Read Register.")

        # Read/Write FIFO Status.
        self._fifo_status  = CSRStatus(description="FIFO Status Register.", fields=[
            CSRField("is_rdempty", size=1, offset=0, description="Read FIFO is empty."),
            CSRField("is_wrfull",  size=1, offset=1, description="Write FIFO is full."),
        ])

        # FIFO Control.
        self._fifo_control = CSRStorage(description="FIFO Control Register.", fields=[
            CSRField("reset", size=1, offset=0, description="Reset Control (Active High).", values=[
                ("``0b0``", "Normal Mode."),
                ("``0b1``", "Reset Mode."),
            ]),
        ])

        # # #

        self.comb += [
            # Read.
            self._fifo_rdata.status.eq(self.ctrl_fifo.rdata),
            self.ctrl_fifo.rd.eq(self._fifo_rdata.we),
            self._fifo_status.fields.is_rdempty.eq(self.ctrl_fifo.empty),
            # Write.
            self.ctrl_fifo.wdata.eq(self._fifo_wdata.storage),
            self.ctrl_fifo.wr.eq(self._fifo_wdata.re),
            self._fifo_status.fields.is_wrfull.eq(self.ctrl_fifo.full),
            # Control.
            self.fifo_reset.eq(self._fifo_control.fields.reset),
        ]
