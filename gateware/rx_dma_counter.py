"""Deterministic, backpressure-safe RX payload for HostTCD hardware tests."""

from migen import Cat, If, Signal
from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litepcie.common import dma_layout


class RXDMACounter(LiteXModule):
    def __init__(self, data_width=256):
        if data_width < 32 or data_width % 32:
            raise ValueError("Counter data width must be a multiple of 32")
        self.enable = Signal()
        self.reset = Signal()
        self.source = source = stream.Endpoint(dma_layout(data_width))
        self.word = word = Signal(32)
        lanes = data_width // 32
        # Least significant lane is the first little-endian word in host RAM.
        self.comb += [
            source.valid.eq(self.enable & ~self.reset),
            source.data.eq(Cat(*[(word + lane)[:32] for lane in range(lanes)])),
            source.first.eq(0), source.last.eq(0),
        ]
        self.sync += If(self.reset,
            word.eq(0)
        ).Elif(source.valid & source.ready,
            word.eq(word + lanes))
