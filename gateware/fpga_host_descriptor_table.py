#
# This file is part of LimeSDR_GW.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *
from migen.genlib.coding import PriorityEncoder

from litex.gen import *
from litex.soc.interconnect import stream, wishbone
from litex.soc.interconnect.csr import *


class FPGAHostDescriptorTable(LiteXModule):
    """Version 2 multi-channel parent table and LitePCIe RX CSR sequencer.

    Each parent describes equal-length channel ranges. Logical blocks visit channels in order;
    each channel block is fragmented at 4 KiB boundaries. Payload must already follow this order.
    With with_payload_commands=True, an ordered, backpressured child command stream is provided
    for a separate channel payload scheduler. This module never separates or timestamps samples.

    Fill parents in program mode, then optionally select loop mode while stopped. Disable drains
    the active parent. Native child descriptors always use program mode. Fatal faults need reset
    of the SoC; a data-starved transfer cannot be aborted through these CSRs.
    """

    VERSION = 2
    INVALID = 1 << 0
    FULL = 1 << 1
    LOOP_WRITE = 1 << 2
    BAD_CHUNK = 1 << 3
    BUS_ERROR = 1 << 4
    UNEXPECTED_RETIRE = 1 << 5
    RESET_BUSY = 1 << 6
    MODE_BUSY = 1 << 7
    MISSING_ADDRESS = 1 << 8
    STAGING_BUSY = 1 << 9
    BAD_INDEX = 1 << 10

    DMA_ENABLE = 0x00
    DMA_VALUE_FLAGS = 0x04
    DMA_VALUE_ADDRESS = 0x08
    DMA_WE = 0x0c
    DMA_LOOP_PROG_N = 0x10
    DMA_RESET = 0x1c

    def __init__(self, dma_base, depth=16, dma_table_depth=256,
                 split_size=2048, alignment=32, bus_timeout=1024,
                 max_channels=16, with_payload_commands=False):
        if not 2 <= depth <= 65535 or dma_table_depth < 1:
            raise ValueError("Invalid descriptor table depth")
        if not 1 <= max_channels <= 16:
            raise ValueError("max_channels must be between 1 and 16")
        if alignment < 4 or alignment & (alignment - 1):
            raise ValueError("Alignment must be a power of two, at least 4 bytes")
        if not 0 < split_size < (1 << 24) or split_size % alignment:
            raise ValueError("Default split size must fit 24 bits and be aligned")
        if dma_base % 4 or bus_timeout < 2:
            raise ValueError("Invalid DMA CSR base or bus timeout")

        self.bus = bus = wishbone.Interface(data_width=32, address_width=32, addressing="word")
        self.child_retired = child_retired = Signal()
        self.irq = Signal()
        slot_bits = bits_for(depth - 1)
        channel_bits = max(1, bits_for(max_channels - 1))

        # Version 2 CSR ABI, 32-bit words. Explicit high/low addresses avoid CSR word-order ambiguity.
        self.version = CSRStatus(32, name="version")                              # 0x00
        self.capabilities = CSRStatus(32, name="capabilities")                    # 0x04
        self.channel_count = CSRStorage(5, reset=1, name="channel_count")         # 0x08
        self.bytes_per_channel = CSRStorage(32, name="bytes_per_channel")         # 0x0c
        self.flags = CSRStorage(2, reset=2, name="flags", description=
            "Bit 0 disables parent IRQ; bit 1 disables early-last termination (required for multi-channel parents).")
        self.address_index = CSRStorage(5, name="address_index")                  # 0x14
        self.address_high = CSRStorage(32, name="address_high")                   # 0x18
        self.address_low = CSRStorage(32, name="address_low")                     # 0x1c
        self.address_write = CSRStorage(1, name="address_write")                  # 0x20
        self.addresses_written = CSRStatus(16, name="addresses_written")          # 0x24
        self.submit = CSRStorage(1, name="submit")                                # 0x28
        self.submit_status = CSRStatus(3, name="submit_status", description=
            "Bit 0: validation/copy busy. Bit 1: last submission accepted. Bit 2: last submission rejected.")
        self.level = CSRStatus(bits_for(depth), name="level")                      # 0x30
        self.enable = CSRStorage(1, name="enable")                                # 0x34
        self.loop_prog_n = CSRStorage(1, name="loop_prog_n", write_from_dev=True)  # 0x38
        self.reset = CSRStorage(1, name="reset")                                  # 0x3c
        self.split_size = CSRStorage(24, reset=split_size, name="split_size")      # 0x40
        self.completed_total = CSRStatus(32, name="completed_total")              # 0x44
        self.loop_status = CSRStatus(32, name="loop_status")                       # 0x48
        self.errors = CSRStatus(32, name="errors")                                # 0x4c
        self.clear_errors = CSRStorage(32, name="clear_errors")                   # 0x50
        self.busy = CSRStatus(1, name="busy")                                     # 0x54
        self.outstanding = CSRStatus(bits_for(dma_table_depth), name="outstanding")
        self.child_issued = CSRStatus(32, name="child_issued")                     # 0x5c
        self.child_completed = CSRStatus(32, name="child_completed")              # 0x60
        self.rejected = CSRStatus(32, name="rejected")                            # 0x64
        self.ring_size = CSRStatus(bits_for(depth), name="ring_size")              # 0x68

        self.active = active = Signal()
        self.armed = armed = Signal()
        self.fatal = fatal = Signal()
        self.parent_address = parent_address = Signal(64)  # Current channel base.
        self.parent_length = parent_length = Signal(32)
        self.next_address = next_address = Signal(64)
        self.remaining = remaining = Signal(32)  # Per-channel bytes left, including current block.
        self.chunk_length = chunk_length = Signal(24)
        self.completed = completed = Signal(32)
        self.issued = issued = Signal()
        self.parent_done = parent_done = Signal()
        self.inflight = inflight = Signal(max=dma_table_depth + 1)
        self.loop_mode = loop_mode = Signal()
        self.loop_index = loop_index = Signal(16)
        self.loop_count = loop_count = Signal(16)
        self.current_channel = channel = Signal(channel_bits)
        self.active_channels = channels = Signal(5)
        self.block_offset = block_offset = Signal(32)
        self.accepted = accepted = Signal()
        self.submitting = submitting = Signal()
        active_slot = Signal(slot_bits)
        ring_size = Signal(max=depth + 1)
        irq_disable = Signal()
        last_disable = Signal()

        # A narrow parent queue holds slot references; addresses are read one channel at a time.
        layout = [("slot", slot_bits), ("length", 32), ("channels", 5),
                  ("irq_disable", 1), ("last_disable", 1)]
        self.fifo = fifo = ResetInserter()(stream.SyncFIFO(layout, depth))
        self.fsm = fsm = FSM(reset_state="OFF")
        self.submit_fsm = sfsm = FSM(reset_state="IDLE")
        used_slots = Signal(depth)
        self.slot_encoder = encoder = PriorityEncoder(depth)
        self.comb += encoder.i.eq(~used_slots)

        # Migen memory ports need at least a one-bit address even for a one-channel build.
        staging = Memory(64, max(2, max_channels))
        staging_wr = staging.get_port(write_capable=True)
        staging_rd = staging.get_port()
        addresses = Memory(64, depth * max_channels)
        addresses_wr = addresses.get_port(write_capable=True)
        addresses_rd = addresses.get_port()
        self.specials += staging, staging_wr, staging_rd, addresses, addresses_wr, addresses_rd

        written = Signal(max_channels)
        validate_channel = Signal(channel_bits)
        reserved_slot = Signal(slot_bits)
        staged_length = Signal(32)
        staged_channels = Signal(5)
        staged_flags = Signal(2)
        last_accepted = Signal()
        last_rejected = Signal()
        reserve = Signal()
        release_reserved = Signal()
        submission_failure = Signal(32)
        reset_requested = self.reset.re & self.reset.storage
        stopped = ~self.enable.storage & fsm.ongoing("OFF")
        reset_ok = reset_requested & stopped & ~submitting
        mode_busy = self.loop_prog_n.re & (~stopped | submitting)
        mode_changed = self.loop_prog_n.re & stopped & ~submitting & (self.loop_prog_n.storage != loop_mode)
        recycling = fsm.ongoing("RECYCLE")
        submit_request = self.submit.re & self.submit.storage
        address_request = self.address_write.re & self.address_write.storage
        stage_writes = (self.channel_count.re | self.bytes_per_channel.re | self.flags.re |
                        self.address_index.re | self.address_high.re | self.address_low.re | address_request)
        basic_invalid = ((self.channel_count.storage == 0) |
                         (self.channel_count.storage > max_channels) |
                         (self.bytes_per_channel.storage == 0) |
                         ((self.bytes_per_channel.storage & (alignment - 1)) != 0) |
                         ((self.channel_count.storage > 1) & ~self.flags.storage[1]))
        checked_address_invalid = (((staging_rd.dat_r & (alignment - 1)) != 0) |
            ((Cat(staging_rd.dat_r, Constant(0, 1)) + staged_length) > (1 << 64)))
        bad_chunk = (self.split_size.storage == 0) | ((self.split_size.storage & (alignment - 1)) != 0)

        self.comb += [
            self.version.status.eq(self.VERSION),
            self.capabilities.status.eq(max_channels | (depth << 8) | (int(with_payload_commands) << 24)),
            submitting.eq(~sfsm.ongoing("IDLE")),
            self.submit_status.status.eq(Cat(submitting, last_accepted, last_rejected)),
            self.addresses_written.status.eq(written),
            staging_wr.adr.eq(self.address_index.storage),
            staging_wr.dat_w.eq(Cat(self.address_low.storage, self.address_high.storage)),
            staging_wr.we.eq(address_request & ~submitting & (self.address_index.storage < max_channels) & ~reset_requested),
            staging_rd.adr.eq(validate_channel),
            addresses_wr.adr.eq(reserved_slot * max_channels + validate_channel),
            addresses_wr.dat_w.eq(staging_rd.dat_r),
            addresses_rd.adr.eq(active_slot * max_channels + channel),
            self.loop_prog_n.we.eq(mode_busy),
            self.loop_prog_n.dat_w.eq(loop_mode),
            fifo.reset.eq(reset_ok),
            fifo.source.ready.eq(parent_done),
            fifo.sink.slot.eq(Mux(recycling, active_slot, reserved_slot)),
            fifo.sink.length.eq(Mux(recycling, parent_length, staged_length)),
            fifo.sink.channels.eq(Mux(recycling, channels, staged_channels)),
            fifo.sink.irq_disable.eq(Mux(recycling, irq_disable, staged_flags[0])),
            fifo.sink.last_disable.eq(Mux(recycling, last_disable, staged_flags[1])),
            If(recycling, fifo.sink.valid.eq(1)),
        ]
        self.sync += [
            If(self.loop_prog_n.re & stopped & ~submitting,
                loop_mode.eq(self.loop_prog_n.storage)),
            If(reset_ok | accepted,
                written.eq(0)
            ).Elif(staging_wr.we,
                written.eq(written | (1 << self.address_index.storage))
            ),
            If(reset_ok,
                used_slots.eq(0), last_accepted.eq(0), last_rejected.eq(0)
            ).Else(
                used_slots.eq((used_slots | Mux(reserve, 1 << encoder.o, 0)) &
                    ~Mux(release_reserved, 1 << reserved_slot, 0) &
                    ~Mux(parent_done & ~loop_mode, 1 << active_slot, 0)),
                If(submit_request & ~submitting,
                    last_accepted.eq(0), last_rejected.eq(0)),
                If(submission_failure != 0, last_rejected.eq(1)),
                If(accepted, last_accepted.eq(1), last_rejected.eq(0)),
            ),
        ]

        # Snapshot scalar fields and lock staging RAM until validation/copy completes.
        sfsm.act("IDLE",
            If(submit_request & ~reset_requested,
                If(loop_mode | self.loop_prog_n.storage,
                    submission_failure.eq(self.LOOP_WRITE)
                ).Elif(basic_invalid,
                    submission_failure.eq(self.INVALID)
                ).Elif(encoder.n,
                    submission_failure.eq(self.FULL)
                ).Else(
                    reserve.eq(1),
                    NextValue(reserved_slot, encoder.o),
                    NextValue(staged_length, self.bytes_per_channel.storage),
                    NextValue(staged_channels, self.channel_count.storage),
                    NextValue(staged_flags, self.flags.storage),
                    NextValue(validate_channel, 0),
                    NextState("READ")
                )
            )
        )
        sfsm.act("READ", NextState("CHECK"))
        sfsm.act("CHECK",
            If(~Array([written[i] for i in range(max_channels)])[validate_channel],
                submission_failure.eq(self.MISSING_ADDRESS),
                release_reserved.eq(1), NextState("IDLE")
            ).Elif(checked_address_invalid,
                submission_failure.eq(self.INVALID),
                release_reserved.eq(1), NextState("IDLE")
            ).Else(
                addresses_wr.we.eq(1),
                If(validate_channel == staged_channels - 1,
                    NextState("ENQUEUE")
                ).Else(
                    NextValue(validate_channel, validate_channel + 1), NextState("READ")
                )
            )
        )
        sfsm.act("ENQUEUE",
            fifo.sink.valid.eq(1),
            If(fifo.sink.ready, accepted.eq(1), NextState("IDLE"))
        )

        # Completion accounting. Only one parent group is active, so aggregate child counts suffice.
        retire = Signal()
        unexpected = Signal()
        bus_failure = Signal()
        error_events = Signal(32)
        error_bits = Signal(32)
        issued_count = Signal(32)
        retired_count = Signal(32)
        rejected_count = Signal(32)
        self.comb += [
            retire.eq(armed & child_retired & ((inflight != 0) | issued)),
            unexpected.eq(armed & child_retired & (inflight == 0) & ~issued),
            error_events.eq(submission_failure |
                Mux(submitting & (stage_writes | submit_request), self.STAGING_BUSY, 0) |
                Mux(address_request & ~submitting & (self.address_index.storage >= max_channels), self.BAD_INDEX, 0) |
                Mux(self.enable.storage & bad_chunk, self.BAD_CHUNK, 0) |
                Mux(bus_failure, self.BUS_ERROR, 0) |
                Mux(unexpected, self.UNEXPECTED_RETIRE, 0) |
                Mux(reset_requested & ~reset_ok, self.RESET_BUSY, 0) |
                Mux(mode_busy, self.MODE_BUSY, 0)),
            self.level.status.eq(fifo.level + recycling),
            self.loop_status.status.eq(Mux(loop_mode, Cat(loop_index, loop_count), completed)),
            self.completed_total.status.eq(completed), self.ring_size.status.eq(ring_size),
            self.busy.status.eq(~fsm.ongoing("OFF")),
            self.outstanding.status.eq(inflight),
            self.child_issued.status.eq(issued_count), self.child_completed.status.eq(retired_count),
            self.rejected.status.eq(rejected_count), self.errors.status.eq(error_bits),
        ]
        self.sync += [
            If(reset_ok,
                completed.eq(0), issued_count.eq(0), retired_count.eq(0),
                rejected_count.eq(0), error_bits.eq(0), inflight.eq(0)
            ).Else(
                error_bits.eq((error_bits & ~Mux(self.clear_errors.re, self.clear_errors.storage, 0)) | error_events),
                If((submission_failure != 0) | (submitting & submit_request),
                    rejected_count.eq(rejected_count + (submission_failure != 0) +
                        (submitting & submit_request))),
                If(issued, issued_count.eq(issued_count + 1)),
                If(retire, retired_count.eq(retired_count + 1)),
                If(issued & ~retire, inflight.eq(inflight + 1)
                ).Elif(retire & ~issued, inflight.eq(inflight - 1)),
                If(parent_done, completed.eq(completed + 1)),
            ),
            If(bus_failure | unexpected, fatal.eq(1)),
            If(reset_ok | mode_changed,
                loop_index.eq(0), loop_count.eq(0), ring_size.eq(0)
            ).Else(
                If(fsm.ongoing("OFF") & self.enable.storage & loop_mode, ring_size.eq(fifo.level)),
                If(parent_done & loop_mode,
                    If(loop_index == ring_size - 1,
                        loop_index.eq(0), loop_count.eq(loop_count + 1)
                    ).Else(loop_index.eq(loop_index + 1)))
            ),
            self.irq.eq(parent_done & ~irq_disable & ~fatal & ~unexpected),
        ]

        wait_count = Signal(max=bus_timeout)
        timeout = Signal()
        self.comb += timeout.eq(wait_count == bus_timeout - 1)
        self.sync += If(bus.cyc & bus.stb & ~bus.ack & ~bus.err,
            If(~timeout, wait_count.eq(wait_count + 1))
        ).Else(wait_count.eq(0))

        # Optional ordered payload commands. A queue slot is reserved before programming a child;
        # no other producer can consume it before the acknowledged native commit.
        command_layout = [("channel", channel_bits), ("channel_count", 5), ("length", 24),
                          ("channel_offset", 32), ("block_offset", 32), ("block_length", 24),
                          ("channel_last", 1), ("block_last", 1), ("sequence", 32)]
        self.command = command = stream.Endpoint(command_layout)
        chunk_limit = Signal(24)
        block_length = Signal(24)
        fragment_offset = Signal(24)
        fragment_remaining = Signal(24)
        page_remaining = Signal(13)
        command_room = Signal()
        commands_drained = Signal()
        if with_payload_commands:
            self.command_fifo = commands = ResetInserter()(stream.SyncFIFO(command_layout, dma_table_depth))
            self.comb += [commands.reset.eq(reset_ok), commands.source.connect(command),
                          command_room.eq(commands.sink.ready), commands_drained.eq(commands.level == 0)]
            command_sink = commands.sink
        else:
            # Diagnostic command pulses; ready is ignored when no payload scheduler is attached.
            command_sink = command
            self.comb += [command_room.eq(1), commands_drained.eq(1)]
        self.comb += [
            command_sink.valid.eq(issued), command_sink.channel.eq(channel),
            command_sink.channel_count.eq(channels), command_sink.length.eq(chunk_length),
            command_sink.channel_offset.eq(block_offset + fragment_offset),
            command_sink.block_offset.eq(block_offset), command_sink.block_length.eq(block_length),
            command_sink.channel_last.eq(fragment_remaining == chunk_length),
            command_sink.block_last.eq((fragment_remaining == chunk_length) & (channel == channels - 1)),
            command_sink.sequence.eq(completed),
            command_sink.first.eq((block_offset == 0) & (channel == 0) & (fragment_offset == 0)),
            command_sink.last.eq((remaining == block_length) & (channel == channels - 1) &
                                 (fragment_remaining == chunk_length)),
            page_remaining.eq(4096 - next_address[:12]),
        ]

        def transaction(name, offset, value, next_state, on_ack=(), read=False, condition=None):
            actions = list(on_ack)
            if condition is None:
                actions.append(NextState(name + "_GAP"))
            else:
                actions = [If(condition, *actions, NextState(name + "_GAP")
                              ).Else(NextState(name + "_RETRY"))]
                fsm.act(name + "_RETRY", NextState(name))
            fsm.act(name,
                bus.cyc.eq(1), bus.stb.eq(1), bus.we.eq(not read), bus.sel.eq(0xf),
                bus.adr.eq((dma_base + offset) >> 2), bus.dat_w.eq(value),
                If(bus.err | (timeout & ~bus.ack), bus_failure.eq(1), NextState("FAULT")
                ).Elif(bus.ack, *actions))
            fsm.act(name + "_GAP", If(fatal, NextState("FAULT")).Else(NextState(next_state)))

        fsm.act("OFF", If(fatal, NextState("FAULT")
            ).Elif(self.enable.storage & ~reset_requested, NextState("DMA_DISABLE")))
        transaction("DMA_DISABLE", self.DMA_ENABLE, 0, "DMA_IDLE")
        transaction("DMA_IDLE", self.DMA_ENABLE, 0, "DMA_RESET", read=True, condition=bus.dat_r[1])
        transaction("DMA_RESET", self.DMA_RESET, 1, "DMA_PROGRAM")
        transaction("DMA_PROGRAM", self.DMA_LOOP_PROG_N, 0, "DMA_ENABLE")
        transaction("DMA_ENABLE", self.DMA_ENABLE, 1, "IDLE", on_ack=[NextValue(armed, 1)])
        fsm.act("IDLE",
            If(fatal, NextState("FAULT")
            ).Elif(~self.enable.storage, NextState("DMA_STOP")
            ).Elif(fifo.source.valid & ~bad_chunk,
                NextValue(active, 1), NextValue(active_slot, fifo.source.slot),
                NextValue(parent_length, fifo.source.length), NextValue(remaining, fifo.source.length),
                NextValue(channels, fifo.source.channels), NextValue(block_offset, 0),
                NextValue(chunk_limit, self.split_size.storage),
                NextValue(irq_disable, fifo.source.irq_disable), NextValue(last_disable, fifo.source.last_disable),
                NextState("NEW_BLOCK")))
        fsm.act("NEW_BLOCK",
            If(fatal, NextState("FAULT")
            ).Elif(remaining == 0, NextState("WAIT_PARENT")
            ).Else(
                NextValue(block_length, Mux(remaining < chunk_limit, remaining, chunk_limit)),
                NextValue(channel, 0), NextValue(fragment_offset, 0), NextState("ADDRESS_READ")))
        fsm.act("ADDRESS_READ", NextState("ADDRESS_LOAD"))
        fsm.act("ADDRESS_LOAD",
            NextValue(parent_address, addresses_rd.dat_r),
            NextValue(next_address, addresses_rd.dat_r + block_offset),
            NextValue(fragment_remaining, block_length), NextState("CHOOSE"))
        fsm.act("CHOOSE",
            If(fatal, NextState("FAULT")
            ).Elif((inflight < dma_table_depth) & command_room,
                NextValue(chunk_length, Mux(fragment_remaining < page_remaining, fragment_remaining, page_remaining)),
                NextState("WRITE_FLAGS")))
        transaction("WRITE_FLAGS", self.DMA_VALUE_FLAGS,
                    Cat(chunk_length, Constant(1, 1), last_disable, Constant(0, 6)), "WRITE_ADDRESS")
        transaction("WRITE_ADDRESS", self.DMA_VALUE_ADDRESS, next_address[:32], "WRITE_COMMIT")
        transaction("WRITE_COMMIT", self.DMA_WE, next_address[32:], "ADVANCE", on_ack=[
            issued.eq(1), NextValue(next_address, next_address + chunk_length),
            NextValue(fragment_remaining, fragment_remaining - chunk_length),
            NextValue(fragment_offset, fragment_offset + chunk_length)])
        fsm.act("ADVANCE",
            If(fatal, NextState("FAULT")
            ).Elif(fragment_remaining != 0, NextState("CHOOSE")
            ).Elif(channel != channels - 1,
                NextValue(channel, channel + 1), NextValue(fragment_offset, 0), NextState("ADDRESS_READ")
            ).Else(
                NextValue(remaining, remaining - block_length),
                NextValue(block_offset, block_offset + block_length), NextState("NEW_BLOCK")))
        fsm.act("WAIT_PARENT",
            If(fatal | unexpected, NextState("FAULT")
            ).Elif((inflight == 0) & commands_drained,
                parent_done.eq(1), NextValue(active, 0),
                If(loop_mode, NextState("RECYCLE")).Else(NextState("IDLE"))))
        fsm.act("RECYCLE", If(fatal, NextState("FAULT")
            ).Elif(fifo.sink.ready, NextState("IDLE")))
        transaction("DMA_STOP", self.DMA_ENABLE, 0, "STOP_IDLE")
        transaction("STOP_IDLE", self.DMA_ENABLE, 0, "OFF", read=True,
                    condition=bus.dat_r[1], on_ack=[NextValue(armed, 0)])
        fsm.act("FAULT", NextState("FAULT"))
