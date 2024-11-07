#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os
from shutil import which, copyfile
import subprocess

from migen        import *
from migen.genlib import fifo

from litex.build import tools

from litex.gen import *

from litex.soc.interconnect import stream

from gateware.busy_delay import BusyDelay
from gateware.common     import FIFOInterface

# Utils --------------------------------------------------------------------------------------------

def FIFORD_SIZE(wr_width, rd_width, wr_size):
    if wr_width > rd_width:
        return int(wr_size+(wr_width/rd_width)/2)
    elif wr_width < rd_width:
        return int(wr_size-(rd_width/wr_width)/2)
    else:
        return wr_size;

# FT601 --------------------------------------------------------------------------------------------

class FT601(LiteXModule):
    def __init__(self, platform, pads=None,
        FT_data_width      = 32,
        FT_be_width        = 4,
        EP02_rdusedw_width = 11,
        EP02_rwidth        = 8,
        EP82_wrusedw_width = 11,
        EP82_wwidth        = 8,
        EP82_wsize         = 64,   # packet size in bytes, has to be multiple of 4 bytes
        EP03_rdusedw_width = 11,
        EP03_rwidth        = 32,
        EP83_wrusedw_width = 12,
        EP83_wwidth        = 64,
        EP83_wsize         = 2048, # packet size in bytes, has to be multiple of 4 bytes
        ):

        assert pads is not None

        self.ctrl_fifo   = FIFOInterface(EP02_rwidth, EP82_wwidth)
        self.stream_fifo = FIFOInterface(EP03_rwidth, EP83_wwidth, EP03_rdusedw_width, EP83_wrusedw_width)

        self.ctrl_fifo_fpga_pc_reset_n   = Signal()
        self.stream_fifo_fpga_pc_reset_n = Signal()
        self.stream_fifo_pc_fpga_reset_n = Signal()

        # # #

        # Signals.
        # --------

        # EP02 fifo signals
        EP02_empty        = Signal()

        # EP82 fifo signals
        EP82_fifo_rdreq   = Signal()

        # EP03 fifo signals
        EP03_empty        = Signal()
        EP03_wr           = Signal()
        EP03_wdata        = Signal(FT_data_width)
        EP03_rdy          = Signal()
        EP03_wr_cnt       = Signal(16)
        EP03_fifo_wusedw  = Signal(11)

        # EP83 fifo signals
        EP83_fifo_rdusedw = Signal(FIFORD_SIZE(EP83_wwidth, FT_data_width, EP83_wrusedw_width))
        EP83_fifo_rdreq   = Signal()
        sync_reg0         = Signal(2)

        # arbiter signals
        arb_en            = Signal()
        arb_rd_wr         = Signal()
        arb_nth_ch        = Signal(4)

        # fsm signals
        fsm_rdy           = Signal()
        fsm_rd_data_valid = Signal()
        fsm_rd_data       = Signal(FT_data_width)
        fsm_wr_data_req   = Signal()
        fsm_wr_data       = Signal(FT_data_width)

        # FTDI endpoint fifos.
        # --------------------

        # Control PC->FPGA FIFO.
        EP02_fifo      = fifo.AsyncFIFO(EP02_rwidth, depth=256)
        self.EP02_fifo = ClockDomainsRenamer({"write":"ft601", "read":"sys"})(EP02_fifo)

        # Control FPGA->PC FIFO.
        EP82_async_fifo      = fifo.AsyncFIFO(EP82_wsize, depth=2)
        self.EP82_async_fifo = ClockDomainsRenamer({"write":"sys", "read":"ft601"})(EP82_async_fifo)
        self.EP82_fifo       = ResetInserter()(ClockDomainsRenamer("ft601")(fifo.SyncFIFO(EP82_wsize, depth=256)))

        # Stream PC->FPGA
        self.EP03_fifo_status = BusyDelay(platform, "ft601",
            clock_period = 10,  #  input clock period in ns
            delay_time   = 100, #  delay time in ms
        )

        if False:
            self.EP03_sink   = stream.Endpoint([("data", FT_data_width)])
            self.EP03_fifo   = ResetInserter()(ClockDomainsRenamer("ft601")(stream.SyncFIFO([("data", FT_data_width)], 1024, True)))
            self.EP03_conv   = ResetInserter()(ClockDomainsRenamer("ft601")(stream.Converter(FT_data_width, EP03_rwidth)))
            self.EP03_cdc    = stream.ClockDomainCrossing([("data", EP03_rwidth)],
                cd_from         = "ft601",
                cd_to           = "lms_tx",
                with_common_rst = True,
            )
            self.EP03_source = stream.Endpoint([("data", EP03_rwidth)])
            self.EP03_pipeline  = stream.Pipeline(
                self.EP03_sink,
                self.EP03_fifo,
                self.EP03_conv,
                self.EP03_cdc,
                self.EP03_source,
            )

            #self.sync.lms_tx += self.EP03_source.ready.eq(self.stream_fifo.rd)
            self.comb += self.EP03_source.ready.eq(self.stream_fifo.rd)

            self.comb += [
                # Reset.
                self.EP03_conv.reset.eq(     ~sync_reg0[1]),
                self.EP03_fifo.reset.eq(     ~sync_reg0[1]),

                self.stream_fifo.rdata.eq(   self.EP03_source.data),
                self.stream_fifo.empty.eq(   ~self.EP03_source.valid),
                self.stream_fifo.rdusedw.eq( self.EP03_fifo.level),

                self.EP03_sink.data.eq( EP03_wdata),
                self.EP03_sink.valid.eq(self.EP03_fifo_status.busy_in),
                EP03_fifo_wusedw.eq(         self.EP03_fifo.level),
            ]
        else:
            self.specials += Instance("fifodc_w32x1024_r128",
                i_Data    = EP03_wdata,
                i_WrClock = ClockSignal("ft601"),
                i_RdClock = ClockSignal("lms_tx"),
                i_WrEn    = self.EP03_fifo_status.busy_in,
                i_RdEn    = self.stream_fifo.rd,
                i_Reset   = ~sync_reg0[1],
                i_RPReset = ~sync_reg0[1],
                o_Q       = self.stream_fifo.rdata,
                o_WCNT    = EP03_fifo_wusedw,
                o_RCNT    = self.stream_fifo.rdusedw,
                o_Empty   = self.stream_fifo.empty,
                o_Full    = Open()
            )

        # Stream FPGA->PC
        EP83_fifo_status = BusyDelay(platform, "ft601",
            clock_period = 10,  #  input clock period in ns
            delay_time   = 100, #  delay time in ms
        )

        # Stream FPGA->PC
        self.EP83_sink = stream.Endpoint([("data", EP83_wwidth)])
        self.EP83_cdc  = stream.ClockDomainCrossing([("data", EP83_wwidth)],
            cd_from         = "lms_rx",
            cd_to           = "ft601",
            with_common_rst = True,
        )
        self.EP83_fifo = ResetInserter()(ClockDomainsRenamer("ft601")(stream.SyncFIFO([("data", EP83_wwidth)], 2024, True)))
        self.EP83_conv = ResetInserter()(ClockDomainsRenamer("ft601")(stream.Converter(EP83_wwidth, FT_data_width)))
        self.EP83_pipeline  = stream.Pipeline(
            self.EP83_sink,
            self.EP83_cdc,
            self.EP83_fifo,
            self.EP83_conv,
        )

        # FTDI arbiter
        self.specials += Instance("FT601_arb",
            # Parameters
            p_FT_data_width     = FT_data_width,
            p_EP82_fifo_rwidth  = FIFORD_SIZE(EP82_wwidth, FT_data_width, EP82_wrusedw_width),
            p_EP82_wsize        = EP82_wsize,
            p_EP83_fifo_rwidth  = FIFORD_SIZE(EP83_wwidth, FT_data_width, EP83_wrusedw_width),
            p_EP83_wsize        = EP83_wsize,

            # Clk/Rst.
            i_clk               = ClockSignal("ft601"),
            i_reset_n           = ~ResetSignal("ft601"),
            i_enable            = Constant(1, 1),

            # Control EP PC->FPGA.
            o_EP02_fifo_data    = self.EP02_fifo.din,
            o_EP02_fifo_wr      = self.EP02_fifo.we,
            i_EP02_fifo_wrempty = ~self.EP02_fifo.readable,

            # Control EP FPGA->PC.
            i_EP82_fifo_data    = self.EP82_fifo.dout,
            o_EP82_fifo_rd      = EP82_fifo_rdreq,
            i_EP82_fifo_rdusedw = self.EP82_fifo.level,

            # Stream EP PC->FPGA.
            o_EP03_fifo_data    = EP03_wdata,
            o_EP03_fifo_wr      = self.EP03_fifo_status.busy_in,
            i_EP03_fifo_wrempty = EP03_rdy,

            # Stream EP FPGA->PC.
            i_EP83_fifo_data    = self.EP83_conv.source.data,
            o_EP83_fifo_rd      = EP83_fifo_status.busy_in,
            i_EP83_fifo_rdusedw = EP83_fifo_rdusedw,

            o_fsm_epgo          = arb_en,
            o_fsm_rdwr          = arb_rd_wr,
            o_fsm_ch            = arb_nth_ch,
            i_fsm_rdy           = fsm_rdy,
            i_fsm_rddata_valid  = fsm_rd_data_valid,
            i_fsm_rddata        = fsm_rd_data,
            i_fsm_wrdata_req    = fsm_wr_data_req,
            o_fsm_wrdata        = fsm_wr_data,
            i_ep_status         = pads.D[8:16],
        )

        #  FTDI fsm
        self.specials += Instance("FT601",
            # Parameters
            p_FT_data_width = FT_data_width,
            p_FT_be_width   = FT_be_width,
            p_EP82_wsize    = EP82_wsize,
            p_EP83_wsize    = EP83_wsize,

            # Clk/Rst.
            i_clk           = ClockSignal("ft601"),
            i_reset_n       = ~ResetSignal("ft601"),

            i_trnsf_en      = arb_en,
            o_ready         = fsm_rdy,
            i_rd_wr         = arb_rd_wr,
            i_ch_n          = arb_nth_ch,

            # FSM.
            o_RD_data_valid = fsm_rd_data_valid,
            o_RD_data       = fsm_rd_data,
            o_WR_data_req   = fsm_wr_data_req,
            i_WR_data       = fsm_wr_data,

            # Physical device.
            o_wr_n          = pads.WRn,
            i_rxf_n         = pads.RXFn,
            io_data         = pads.D,
            io_be           = pads.BE,
            i_txe_n         = pads.TXEn,
        )

        # Logic.
        # ------
        self.comb += [
            # EP02 EP.
            # --------
            self.EP02_fifo.re.eq(         self.ctrl_fifo.rd),
            self.ctrl_fifo.rdata.eq(      self.EP02_fifo.dout),
            self.ctrl_fifo.empty.eq(      ~self.EP02_fifo.readable),

            # EP82 EP.
            # --------
            # Reset
            self.EP82_fifo.reset.eq(      ~self.ctrl_fifo_fpga_pc_reset_n),
            # Control -> AsyncFIFO
            self.EP82_async_fifo.din.eq(   self.ctrl_fifo.wdata),
            self.EP82_async_fifo.we.eq(    self.ctrl_fifo.wr),
            self.ctrl_fifo.full.eq(        ~self.EP82_async_fifo.writable),
            # AsyncFIFO -> SyncFIFO
            self.EP82_fifo.din.eq(         self.EP82_async_fifo.dout),
            self.EP82_fifo.we.eq(          self.EP82_async_fifo.readable),
            self.EP82_async_fifo.re.eq(    self.EP82_fifo.writable),

            # EP83 Fifo.
            # ----------
            # Reset
            self.EP83_fifo.reset.eq(~self.stream_fifo_fpga_pc_reset_n),
            self.EP83_conv.reset.eq(~self.stream_fifo_fpga_pc_reset_n),
            # Fifo Interface -> stream.Endpoint
            self.EP83_sink.data.eq(self.stream_fifo.wdata),
            self.EP83_sink.valid.eq(self.stream_fifo.wr),
            self.stream_fifo.full.eq(~self.EP83_sink.ready),
            self.stream_fifo.wrusedw.eq(self.EP83_fifo.level),
            EP83_fifo_rdusedw.eq(Cat(0, self.EP83_fifo.level)),

            self.stream_fifo.wr_active.eq(EP83_fifo_status.busy_out),
            pads.RESETn.eq(~ResetSignal("sys")),
            self.stream_fifo.rd_active.eq(self.EP03_fifo_status.busy_out),
        ]

        self.sync.ft601 += [
            # FIXME: policy with lattice (and maybe altera) is to pop data before reading
            # (SyncFIFOBuffered not working too).
            # EP82 Fifo.
            # ----------
            self.EP82_fifo.re.eq(EP82_fifo_rdreq),

            # EP83 Fifo.
            # ----------
            self.EP83_conv.source.ready.eq(EP83_fifo_status.busy_in),

            If(self.stream_fifo_pc_fpga_reset_n == 0b0,
                sync_reg0.eq(0b00),
            ).Else(
                sync_reg0.eq(Cat(1, sync_reg0[0]))
            ),
            If(self.EP03_fifo_status.busy_in == 0b1,
                EP03_wr_cnt.eq(EP03_wr_cnt+1),
            ).Else(
                EP03_wr_cnt.eq(0),
            ),
            If(self.stream_fifo.empty == 0b0 or EP03_fifo_wusedw > 0,
                EP03_rdy.eq(0),
            ).Else(
                EP03_rdy.eq(1),
            )
        ]

        self.add_sources(platform)

    def add_sources(self, platform):
        ft601_files = [
            "LimeSDR-Mini_lms7_trx/src/FT601/synth/FT601_arb.vhd",
            "LimeSDR-Mini_lms7_trx/src/FT601/synth/FT601.vhd",
            "LimeSDR-Mini_lms7_trx/proj//ip/fifodc_w32x1024_r128/fifodc_w32x1024_r128.vhd",
        ]

        for file in ft601_files:
            platform.add_source(file)
