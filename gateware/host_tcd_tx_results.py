# SPDX-License-Identifier: Apache-2.0
"""Ordered chunk outcomes aggregated into bounded per-parent completion records."""
from migen import *
from litex.gen import LiteXModule
from litex.soc.interconnect import stream

OUTCOME = [('sequence',32), ('samples',32), ('transmitted',32), ('offset',32),
           ('timestamp',64), ('parent_last',1), ('irq_disable',1), ('late',1),
           ('failed',1), ('parent_samples',32)]
RECORD = [('sequence',32), ('requested',32), ('transmitted',32), ('skipped',32),
          ('flags',32), ('timestamp_low',32), ('timestamp_high',32), ('chunks',32)]


class TXParentResults(LiteXModule):
    def __init__(self, chunk_samples, depth=16):
        self.sink = r = stream.Endpoint(OUTCOME)
        self.fifo = fifo = ResetInserter()(stream.SyncFIFO(RECORD, depth, buffered=True))
        self.source = fifo.source
        self.done = Signal()
        self.irq = Signal()
        self.error = Signal()
        self.clear = Signal()
        self.comb += fifo.reset.eq(self.clear)
        sent = Signal(32)
        skipped = Signal(32)
        flags = Signal(32)
        chunks = Signal(32)
        expected = Signal(32)
        nsent = Signal(32)
        nskip = Signal(32)
        nflags = Signal(32)
        nchunks = Signal(32)
        base_tick = Signal(64)
        self.comb += [
            nsent.eq(sent+r.transmitted), nskip.eq(skipped+r.samples-r.transmitted),
            nflags.eq(flags | Cat(r.late, r.failed)), nchunks.eq(chunks+(r.samples >> log2_int(chunk_samples))),
            base_tick.eq(r.timestamp-r.offset), r.ready.eq(~r.parent_last | fifo.sink.ready),
            fifo.sink.valid.eq(r.valid & r.parent_last), fifo.sink.sequence.eq(r.sequence),
            fifo.sink.requested.eq(r.parent_samples), fifo.sink.transmitted.eq(nsent),
            fifo.sink.skipped.eq(nskip), fifo.sink.flags.eq(nflags), fifo.sink.chunks.eq(nchunks),
            fifo.sink.timestamp_low.eq(base_tick[:32]), fifo.sink.timestamp_high.eq(base_tick[32:]),
            self.done.eq(fifo.sink.valid & fifo.sink.ready), self.irq.eq(self.done & ~r.irq_disable)]
        self.sync += If(r.valid & r.ready,
            If((r.sequence != expected) | (r.transmitted > r.samples), self.error.eq(1)),
            If(r.parent_last,
                If((nsent+nskip) != r.parent_samples, self.error.eq(1)),
                sent.eq(0), skipped.eq(0), flags.eq(0), chunks.eq(0), expected.eq(expected+1)
            ).Else(sent.eq(nsent), skipped.eq(nskip), flags.eq(nflags), chunks.eq(nchunks)))
        self.sync += If(self.clear, sent.eq(0),skipped.eq(0),flags.eq(0),chunks.eq(0),expected.eq(0),self.error.eq(0))
