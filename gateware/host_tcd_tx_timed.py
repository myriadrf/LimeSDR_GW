# SPDX-License-Identifier: Apache-2.0
"""Configurable buffered TX segments with independent DMA and playback ownership."""
from migen import *
from migen.genlib.coding import PriorityEncoder
from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRStatus, CSRStorage
from gateware.host_tcd_tx_results import OUTCOME, RECORD, TXParentResults
from gateware.host_tcd_tx_diagnostics import DIAGNOSTIC, TXDiagnostics


class TXTimedBuffer(LiteXModule):
    def __init__(self, command_layout, samples=1024, max_channels=4, playback_domain="afe", streaming=False, slots=2,
                 distributed_lanes=0):
        if not 0 <= distributed_lanes <= 8:
            raise ValueError("Distributed lanes must be from 0 through 8")
        if not 2 <= slots <= 16:
            raise ValueError("TX slot count must be from 2 through 16")
        slot_bits = log2_int(slots, need_pow2=False)
        all_free = (1 << slots)-1
        if samples < 32 or samples & (samples - 1) or not 1 <= max_channels <= 4:
            raise ValueError("TX storage requires a power-of-two capacity >=32 and 1..4 channels")
        self.parent = p = stream.Endpoint([
            ("length", 32), ("channels", 5), ("timestamp", 64), ("iq12", 1), ("irq_disable", 1)] +
            ([("samples",32), ("offset",32), ("sequence",32), ("parent_last",1),
              ("skip",1), ("parent_samples",32)] if streaming else []))
        self.command = c = stream.Endpoint(command_layout)
        self.sink = s = stream.Endpoint([("data", 256)])
        self.source = o = stream.Endpoint([("data", 128)])
        self.rx_timestamp = Signal(64)
        self.sample_tick = Signal(reset=1)
        self.retired = Signal()
        self.loaded = Signal()  # Current fetch published; held until next admission.
        self.done = Signal()    # One sys pulse per playback/discard, not per fetch.
        self.irq = Signal()
        self.version = CSRStatus(32, reset=2 if streaming else 1, name="version")
        self.completed = CSRStatus(32, name="completed")
        self.late = CSRStatus(32, name="late")
        self.underrun = CSRStatus(32, name="underrun")
        self.busy = CSRStatus(1, name="busy")
        self.slots = CSRStatus(32, reset=slots, name="slots")  # Appended at 0x14.
        self.occupied = CSRStatus(slots, name="occupied")
        self.unpack_lanes = CSRStatus(32, reset=8, name="unpack_lanes")
        self.now_sys = Signal(64)
        self.clear_results = Signal()  # Only while both DMA and sample slots are idle.
        if streaming:
            # FIFO snapshots transfer the native counter coherently, not as 64
            # independently synchronized bits. preparation_ticks covers CDC age.
            self.time_cdc = tc = stream.ClockDomainCrossing([('tick',64)],
                cd_from=playback_domain, cd_to='sys', depth=4, buffered=True)
            self.comb += [tc.sink.valid.eq(1), tc.sink.tick.eq(self.rx_timestamp), tc.source.ready.eq(1)]
            self.sync += If(tc.source.valid, self.now_sys.eq(tc.source.tick))
            self.results = results = TXParentResults(samples)
            self.comb += results.clear.eq(self.clear_results)
            self.record_available = CSRStatus(1, name='record_available')  # 0x20
            self.record_snapshot = CSRStorage(1, name='record_snapshot')  # 0x24
            self.snapshot_valid = CSRStatus(1, name='snapshot_valid')      # 0x28
            for i in range(8): setattr(self, 'record_word'+str(i), CSRStatus(32,name='record_word'+str(i)))
            self.record_pop = CSRStorage(1, name='record_pop')            # 0x4c
            self.protocol_error = CSRStatus(1, name='protocol_error')     # 0x50
            self.comb += [self.record_available.status.eq(results.source.valid),
                self.protocol_error.status.eq(results.error),
                results.source.ready.eq(self.record_pop.re & self.record_pop.storage & self.snapshot_valid.status)]
            self.sync += [
                If(self.record_snapshot.re & self.record_snapshot.storage & results.source.valid,
                    self.snapshot_valid.status.eq(1),
                    *[getattr(self,'record_word'+str(i)).status.eq(getattr(results.source,name))
                      for i,(name,_) in enumerate(RECORD)]),
                If(results.source.valid & results.source.ready, self.snapshot_valid.status.eq(0))]
            self.sync += If(self.clear_results, self.snapshot_valid.status.eq(0))
            self.diagnostics = diag = TXDiagnostics()
            self.diag_version = CSRStatus(32, reset=1, name='diag_version')  # 0x54
            self.diag_enable = CSRStorage(1, name='diag_enable')            # 0x58
            self.diag_events = CSRStatus(32, name='diag_events')            # 0x5c
            self.diag_overflow = CSRStatus(32, name='diag_overflow')        # 0x60
            self.diag_level = CSRStatus(32, name='diag_level')              # 0x64
            self.diag_snapshot = CSRStorage(1, name='diag_snapshot')        # 0x68
            self.diag_valid = CSRStatus(1, name='diag_valid')                # 0x6c
            for i in range(17):
                setattr(self, 'diag_word'+str(i), CSRStatus(32, name='diag_word'+str(i)))
            self.diag_pop = CSRStorage(1, name='diag_pop')                  # 0xb4
            self.diag_clear = CSRStorage(1, name='diag_clear')              # 0xb8
            self.diag_min_valid = CSRStatus(1, name='diag_min_valid')        # 0xbc
            self.diag_min_low = CSRStatus(32, name='diag_min_low')           # 0xc0
            self.diag_min_high = CSRStatus(32, name='diag_min_high')         # 0xc4
            packed_diag = Cat(*[getattr(diag.source, name) for name, _ in DIAGNOSTIC])
            self.comb += [diag.enable.eq(self.diag_enable.storage),
                diag.clear.eq(self.clear_results | (self.diag_clear.re & self.diag_clear.storage)),
                self.diag_events.status.eq(diag.events), self.diag_overflow.status.eq(diag.overflow),
                self.diag_level.status.eq(diag.level),
                diag.source.ready.eq(self.diag_pop.re & self.diag_pop.storage & self.diag_valid.status)]
            self.sync += [
                If(self.diag_snapshot.re & self.diag_snapshot.storage,
                    self.diag_min_valid.status.eq(diag.minimum_valid),
                    self.diag_min_low.status.eq(diag.minimum[:32]),
                    self.diag_min_high.status.eq(diag.minimum[32:])),
                If(self.diag_snapshot.re & self.diag_snapshot.storage & diag.source.valid,
                    self.diag_valid.status.eq(1),
                    *[getattr(self, 'diag_word'+str(i)).status.eq(packed_diag[32*i:32*(i+1)])
                      for i in range(17)]),
                If(diag.source.valid & diag.source.ready, self.diag_valid.status.eq(0)),
                If(diag.clear, self.diag_valid.status.eq(0))]

        active = Signal()
        length = Signal(32)
        channels = Signal(5)
        timestamp = Signal(64)
        iq12 = Signal()
        irq_disable = Signal()
        load_slot = Signal(slot_bits)
        self.free = free = Signal(slots, reset=all_free)
        selected = Signal(slot_bits)
        self.slot_encoder = encoder = PriorityEncoder(slots)
        reserve = p.valid & p.ready
        self.comb += [encoder.i.eq(free), selected.eq(encoder.o), p.ready.eq(~active & (free != 0)),
            self.busy.status.eq(free != all_free), self.occupied.status.eq(all_free ^ free)]
        child_active = Signal()
        child_channel = Signal(2)
        child_left = Signal(24)
        received = Signal(32)
        decoded = Signal(32)
        sealed = Signal()
        skip = Signal()
        extra = {}
        if streaming:
            extra = {name:Signal(width) for name,width in p.description.payload_layout
                if name in ('samples','offset','sequence','parent_last','parent_samples')}
            self.sync += If(reserve, skip.eq(p.skip),
                *[value.eq(getattr(p,name)) for name,value in extra.items()])
            admitted_ticks = [Signal(64) for _ in range(slots)]
            command_ticks = [Signal(64) for _ in range(slots)]
            response_ticks = [Signal(64) for _ in range(slots)]
            ready_ticks = [Signal(64) for _ in range(slots)]
            timing_valid = [Signal(3) for _ in range(slots)]
            for slot in range(slots):
                self.sync += If(reserve & (selected == slot),
                    admitted_ticks[slot].eq(self.now_sys), timing_valid[slot].eq(0),
                    command_ticks[slot].eq(0), response_ticks[slot].eq(0), ready_ticks[slot].eq(0)
                ).Else(
                    If(c.valid & c.ready & (load_slot == slot) & ~timing_valid[slot][0],
                        command_ticks[slot].eq(self.now_sys), timing_valid[slot][0].eq(1)),
                    If(s.valid & s.ready & (load_slot == slot) & (received+32 == length*channels),
                        response_ticks[slot].eq(self.now_sys), timing_valid[slot][1].eq(1)),
                    If(active & sealed & ~skip & (load_slot == slot) & ~timing_valid[slot][2],
                        ready_ticks[slot].eq(self.now_sys), timing_valid[slot][2].eq(1)))
        self.comb += c.ready.eq(active & ~child_active & ~sealed & ~skip)
        self.sync += [
            If(reserve, active.eq(1), load_slot.eq(selected), length.eq(p.length), channels.eq(p.channels),
                timestamp.eq(p.timestamp), iq12.eq(p.iq12), irq_disable.eq(p.irq_disable),
                received.eq(0), decoded.eq(0), sealed.eq(0), self.loaded.eq(0)),
            If(c.valid & c.ready, child_active.eq(1), child_channel.eq(c.channel), child_left.eq(c.length)),
            If(s.valid & s.ready, received.eq(received + 32), child_left.eq(child_left - 32),
                If(child_left == 32, child_active.eq(0))),
        ]
        # Separate RAM read ports keep the other slot's first sample primed.
        # This permits back-to-back playback without a RAM read-latency bubble.
        # Decode eight IQ pairs per write. Packed IQ12 retains residual bits
        # across DMA beats; 448 bits covers a residual group plus a 256-bit beat.
        reservoirs = [Signal(448) for _ in range(max_channels)]
        # Both a 192-bit group and a 256-bit beat are multiples of 64.
        # Track occupancy in those units to keep barrel shifts and control small.
        counts = [Signal(3) for _ in range(max_channels)]
        positions = [Signal(max=samples + 1) for _ in range(max_channels)]
        emits = [Signal() for _ in range(max_channels)]
        bits = Signal(9)
        units = Signal(3)
        after_emit = Signal(3)
        self.comb += [units.eq(Mux(iq12, 3, 4)), bits.eq(units << 6),
            after_emit.eq(Array(counts)[child_channel] - Mux(Array(emits)[child_channel], units, 0)),
            s.ready.eq(child_active & (after_emit <= 3)),
            self.retired.eq(s.valid & s.ready & (child_left == 32))]
        metadata = [("timestamp", 64), ("channels", 5), ("samples", 32 if streaming else len(positions[0])),
                    ("slot", slot_bits), ("irq_disable", 1)] + ([("skip",1), ("offset",32),
                    ("sequence",32), ("parent_last",1), ("parent_samples",32)] if streaming else [])
        self.metadata = meta = stream.ClockDomainCrossing(metadata,
            cd_from="sys", cd_to=playback_domain, depth=4, buffered=streaming)
        m = meta.source
        play_index = Signal(max=samples)
        running = Signal()
        advance = Signal()
        next_index = Signal(max=samples)
        # The timestamp compare must not feed the RAM address path. Starting
        # always reads lane 0; subsequent lookahead depends on registered running.
        self.comb += next_index.eq(Mux(running & self.sample_tick & o.ready, play_index + 1, play_index))
        outputs = []
        writes = []
        for ch in range(max_channels):
            emit = emits[ch]
            self.comb += emit.eq(active & (counts[ch] >= units) & (positions[ch] < samples))
            read_data = []
            for slot in range(slots):
                lanes = []
                for lane in range(8):
                    ram = Memory(32, samples // 8)
                    # Move a bounded part of the last slot to LUTRAM on small
                    # devices without changing capacity, clocks or read latency.
                    # The board's LiteXMemoryPatcher emits this explicit style.
                    if slot == slots-1 and lane < distributed_lanes:
                        ram.ram_style = "distributed"
                    wr = ram.get_port(write_capable=True)
                    rd = ram.get_port(clock_domain=playback_domain)
                    self.specials += ram, wr, rd
                    self.comb += [wr.we.eq(emit & (load_slot == slot)), wr.adr.eq(positions[ch][3:]),
                        wr.dat_w.eq(Mux(iq12,
                            Cat(Constant(0, 4), reservoirs[ch][lane*24:lane*24+12],
                                Constant(0, 4), reservoirs[ch][lane*24+12:lane*24+24]),
                            reservoirs[ch][lane*32:lane*32+32])),
                        rd.adr.eq(Mux(m.slot == slot, next_index[3:], 0))]
                    lanes.append(rd.dat_r)
                read_data.append(Array(lanes)[play_index[:3]])
            outputs.append(Mux(m.channels > ch, Array(read_data)[m.slot], 0))
            writes.append(emit)
            incoming = s.valid & s.ready & (child_channel == ch)
            remaining_units = Signal(3)
            self.comb += remaining_units.eq(counts[ch] - Mux(emit, units, 0))
            self.sync += If(reserve, reservoirs[ch].eq(0), counts[ch].eq(0), positions[ch].eq(0)
                ).Else(
                    If(emit, positions[ch].eq(positions[ch] + 8)),
                    If(emit | incoming,
                        reservoirs[ch].eq(Mux(emit, reservoirs[ch] >> bits, reservoirs[ch]) |
                            Mux(incoming, s.data << (remaining_units << 6), 0)),
                        counts[ch].eq(remaining_units + Mux(incoming, 4, 0))))
        self.sync += [If(~reserve, decoded.eq(decoded + sum(writes)*8)),
            If(active & (received == length * channels) &
                (decoded * Mux(iq12, 3, 4) == length * channels), sealed.eq(1))]
        self.comb += [meta.sink.valid.eq(active & (sealed | skip)), meta.sink.timestamp.eq(timestamp),
            meta.sink.channels.eq(channels), meta.sink.samples.eq(Mux(skip, extra['samples'], positions[0]) if streaming else positions[0]),
            meta.sink.slot.eq(load_slot), meta.sink.irq_disable.eq(irq_disable)]
        if streaming:
            self.comb += [meta.sink.skip.eq(skip), *[getattr(meta.sink,name).eq(value)
                for name,value in extra.items() if name != 'samples']]
            # Pipeline the diagnostic subtraction; it never feeds scheduling.
            self.sync += [diag.margin_valid.eq(meta.sink.valid & meta.sink.ready & ~skip),
                diag.margin.eq(timestamp-self.now_sys)]
        self.sync += If(meta.sink.valid & meta.sink.ready, active.eq(0), self.loaded.eq(1))

        # Return slot identity and outcome atomically. Unlike pulse synchronizers,
        # a queue cannot lose consecutive drops when playback is faster than sys.
        self.returns = returns = stream.ClockDomainCrossing([("slot",slot_bits)] + (OUTCOME + [('decision_tick',64), ('reason',3)] if streaming else [
            ("late", 1), ("failed", 1), ("irq_disable", 1)]),
            cd_from=playback_domain, cd_to="sys", depth=4, buffered=False)
        r = returns.source
        if streaming:
            self.comb += r.connect(results.sink, omit={'slot', 'decision_tick', 'reason'})
            self.comb += [diag.sink.valid.eq(r.valid & r.ready & (r.late | r.failed)),
                diag.sink.sequence.eq(r.sequence), diag.sink.offset.eq(r.offset),
                diag.sink.samples.eq(r.samples), diag.sink.transmitted.eq(r.transmitted),
                diag.sink.info.eq(r.reason | (r.slot << 8) | (Array(timing_valid)[r.slot] << 16)),
                diag.sink.scheduled.eq(r.timestamp), diag.sink.admitted.eq(Array(admitted_ticks)[r.slot]),
                diag.sink.command.eq(Array(command_ticks)[r.slot]),
                diag.sink.response.eq(Array(response_ticks)[r.slot]),
                diag.sink.ready_tick.eq(Array(ready_ticks)[r.slot]),
                diag.sink.decision.eq(Mux(r.reason == 1, Array(admitted_ticks)[r.slot], r.decision_tick))]
            done, irq = results.done, results.irq
            late_event, under_event = results.fifo.sink.flags[0], results.fifo.sink.flags[1]
        else:
            self.comb += r.ready.eq(1)
            done, irq = r.valid, r.valid & ~r.irq_disable
            late_event, under_event = r.late, r.failed
        self.sync += [self.done.eq(done), self.irq.eq(irq),
            free.eq((free & ~Mux(reserve, 1 << selected, 0)) | Mux(r.valid & r.ready, 1 << r.slot, 0)),
            If(done, self.completed.status.eq(self.completed.status + 1),
                If(late_event, self.late.status.eq(self.late.status + 1)),
                If(under_event, self.underrun.status.eq(self.underrun.status + 1)))]
        # Parallel 16-bit comparisons avoid a 64-bit subtract/carry chain in
        # the AFE scheduling control path. Modular order assumes <2**63 horizon.
        greater, less = Constant(0), Constant(0)
        equal_high = Constant(1)
        for lo, hi in [(48,63),(32,48),(16,32),(0,16)]:
            a, z = self.rx_timestamp[lo:hi], m.timestamp[lo:hi]
            greater = greater | (equal_high & (a > z))
            less = less | (equal_high & (a < z))
            equal_high = equal_high & (a == z)
        after = Mux(self.rx_timestamp[63] == m.timestamp[63], greater, less)
        due = Signal()
        start = Signal()
        late = Signal()
        failed = Signal()
        finish = Signal()
        skip_now = m.skip if streaming else Constant(0)
        self.comb += [due.eq(self.rx_timestamp == m.timestamp),
            start.eq(m.valid & ~running & due & o.ready & self.sample_tick & ~skip_now),
            o.valid.eq(m.valid & (running | start) & self.sample_tick & ~skip_now), o.data.eq(Cat(*outputs)),
            advance.eq(o.valid & o.ready),
            late.eq(skip_now | (~running & ((after & ~due) | (due & self.sample_tick & ~o.ready)))),
            failed.eq(running & self.sample_tick & ~o.ready),
            finish.eq(m.valid & (late | failed | (advance & (play_index == m.samples - 1)))),
            returns.sink.valid.eq(finish), returns.sink.slot.eq(m.slot),
            returns.sink.late.eq(late), returns.sink.failed.eq(failed),
            returns.sink.irq_disable.eq(m.irq_disable), m.ready.eq(finish & returns.sink.ready)]
        if streaming:
            decision_tick = Signal(64)
            decision_reason = Signal(3)
            decision_pending = Signal()
            reason = Mux(m.skip, 1, Mux(failed, 4, Mux(due & ~o.ready, 3, 2)))
            psync_diag = getattr(self.sync, playback_domain)
            psync_diag += If(m.valid & m.ready, decision_pending.eq(0)
                ).Elif(finish & ~decision_pending, decision_tick.eq(self.rx_timestamp),
                    decision_reason.eq(reason), decision_pending.eq(1))
            self.comb += [returns.sink.decision_tick.eq(Mux(decision_pending, decision_tick, self.rx_timestamp)),
                returns.sink.reason.eq(Mux(decision_pending, decision_reason, reason))]
            self.comb += [returns.sink.samples.eq(m.samples), returns.sink.offset.eq(m.offset),
                returns.sink.sequence.eq(m.sequence), returns.sink.timestamp.eq(m.timestamp),
                returns.sink.parent_last.eq(m.parent_last), returns.sink.parent_samples.eq(m.parent_samples),
                returns.sink.transmitted.eq(Mux(late, 0, Mux(failed, play_index, m.samples)))]
        psync = getattr(self.sync, playback_domain)
        psync += If(m.valid & m.ready, running.eq(0), play_index.eq(0)
            ).Elif(advance, running.eq(1), play_index.eq(play_index + 1))
