# SPDX-License-Identifier: Apache-2.0
"""Diagnostic PCIe fetch sink. No samples are forwarded to the AFE."""
from migen import *
from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStorage, CSRStatus


class TXFetchChecker(LiteXModule):
    def __init__(self, command_layout):
        self.command = c = stream.Endpoint(command_layout)
        self.sink = s = stream.Endpoint([('data',256)])
        self.retired = Signal()
        self.idle = Signal()
        self.version = CSRStatus(32, name='version')
        self.clear = CSRStorage(1, name='clear')
        self.words = CSRStatus(32, name='words')
        self.mismatches = CSRStatus(32, name='mismatches')
        self.children = CSRStatus(32, name='children')
        self.first_expected = CSRStatus(32, name='first_expected')
        self.first_actual = CSRStatus(32, name='first_actual')
        self.busy = CSRStatus(1, name='busy')
        active = Signal()
        remaining = Signal(24)
        expected = Signal(32)
        count = Signal(32)
        errors = Signal(32)
        children = Signal(32)
        first = Signal()
        # Buffer metadata before using it; ready depends only on occupancy.
        self.pipe = pipe = stream.Buffer(command_layout, pipe_ready=True)
        self.comb += c.connect(pipe.sink)
        m = pipe.source
        self.comb += [self.version.status.eq(1),m.ready.eq(~active),s.ready.eq(active),
            self.retired.eq(s.valid & s.ready & (remaining == 32)),
            self.words.status.eq(count),self.mismatches.status.eq(errors),
            self.children.status.eq(children),self.busy.status.eq(active)]
        # Host pattern: channel*0x10000000 + word index within its buffer.
        mismatch = Signal(8)
        self.comb += mismatch.eq(Cat(*[s.data[i*32:(i+1)*32] != expected+i for i in range(8)]))
        self.sync += [
            If(m.valid & m.ready,active.eq(1),remaining.eq(m.length),
                expected.eq((m.channel << 28) + (m.channel_offset >> 2))),
            If(s.valid & s.ready,
                count.eq(count+8),expected.eq(expected+8),remaining.eq(remaining-32),
                If(mismatch != 0,
                    errors.eq(errors+1),
                    If(~first,first.eq(1),self.first_expected.status.eq(expected),
                        self.first_actual.status.eq(s.data[:32]))),
                If(remaining == 32,active.eq(0),children.eq(children+1))),
            If(self.clear.re & self.clear.storage & self.idle,
                count.eq(0),errors.eq(0),children.eq(0),first.eq(0),
                self.first_expected.status.eq(0),self.first_actual.status.eq(0))]
