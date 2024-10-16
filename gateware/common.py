#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

class FIFOInterface(LiteXModule):
    def __init__(self, rdata_width, wdata_wdith, rdusedw_size=1, wrusedw_size=1):
        self.rd          = Signal()
        self.wr          = Signal()
        self.rd_active   = Signal()
        self.wr_active   = Signal()
        self.rdata       = Signal(rdata_width)
        self.wdata       = Signal(wdata_wdith)
        self.empty       = Signal()
        self.full        = Signal()
        self.rdusedw     = Signal(rdusedw_size)
        self.wrusedw     = Signal(wrusedw_size)

    def connect(self, fifo_if):
        return [
            fifo_if.rd.eq(self.rd),
            fifo_if.wr.eq(self.wr),
            self.rd_active.eq(fifo_if.rd_active),
            self.wr_active.eq(fifo_if.wr_active),
            self.rdata.eq(fifo_if.rdata),
            fifo_if.wdata.eq(self.wdata),
            self.full.eq(fifo_if.full),
            self.empty.eq(fifo_if.empty),
            self.rdusedw.eq(fifo_if.rdusedw),
            self.wrusedw.eq(fifo_if.wrusedw),
        ]
