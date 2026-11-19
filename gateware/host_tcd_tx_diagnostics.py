# SPDX-License-Identifier: Apache-2.0
"""Lossy TX diagnostics: overflow never backpressures the sample path."""
from migen import *
from litex.gen import LiteXModule
from litex.soc.interconnect import stream

DIAGNOSTIC = [('sequence', 32), ('offset', 32), ('samples', 32),
              ('transmitted', 32), ('info', 32)] + [
    (name, 64) for name in ('scheduled', 'admitted', 'command', 'response', 'ready_tick', 'decision')]


class TXDiagnostics(LiteXModule):
    def __init__(self, depth=64):
        self.sink = s = stream.Endpoint(DIAGNOSTIC)
        self.enable = Signal()
        self.clear = Signal()
        self.events = Signal(32)
        self.overflow = Signal(32)
        self.margin = Signal((64, True))
        self.margin_valid = Signal()
        self.minimum = Signal((64, True))
        self.minimum_valid = Signal()
        self.fifo = fifo = ResetInserter()(stream.SyncFIFO(DIAGNOSTIC, depth, buffered=True))
        self.source = fifo.source
        self.level = fifo.level
        self.comb += [fifo.reset.eq(self.clear), s.ready.eq(1),
            fifo.sink.valid.eq(s.valid & self.enable & ~self.clear),
            *[getattr(fifo.sink, name).eq(getattr(s, name)) for name, _ in DIAGNOSTIC]]
        self.sync += If(s.valid & self.enable,
            self.events.eq(self.events + 1),
            If(~fifo.sink.ready, self.overflow.eq(self.overflow + 1)))
        self.sync += If(self.margin_valid & self.enable,
            If(~self.minimum_valid | (self.margin < self.minimum), self.minimum.eq(self.margin)),
            self.minimum_valid.eq(1))
        self.sync += If(self.clear, self.events.eq(0), self.overflow.eq(0),
            self.minimum.eq(0), self.minimum_valid.eq(0))
