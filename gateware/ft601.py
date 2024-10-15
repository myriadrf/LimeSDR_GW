#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

import os
from shutil import which, copyfile
import subprocess

from migen import *

from litex.build import tools

from litex.gen import *

from litex.soc.interconnect import stream

# FT601 --------------------------------------------------------------------------------------------

class FT601(LiteXModule):
    def __init__(self, platform, pads=None, clk_pads=None,
        FT_data_width      = 31,
        FT_be_width        = 4,
        EP02_rdusedw_width = 11,
        EP02_rwidth        = 8,
        EP82_wrusedw_width = 11,
        EP82_wwidth        = 8,
        EP82_wsize         = 64,
        EP03_rdusedw_width = 11,
        EP03_rwidth        = 32,
        EP83_wrusedw_width = 12,
        EP83_wwidth        = 64,
        EP83_wsize         = 2048,
        ):

        assert pads is not None
        assert clk_pads is not None

        self.ctrl_fifo_pc_fpga           = stream.Endpoint([("data", EP02_rwidth), ("empty", 1)])
        self.ctrl_fifo_fpga_pc           = stream.Endpoint([("data", EP82_wwidth), ("full", 1)])
        self.stream_fifo_pc_fpga         = stream.Endpoint([("data", EP03_rwidth), ("active", 1), ("empty", 1), ("usedw", EP03_rdusedw_width)])
        self.stream_fifo_fpga_pc         = stream.Endpoint([("data", EP83_wwidth), ("active", 1), ("full", 1),  ("usedw", EP83_wrusedw_width)])

        self.ctrl_fifo_fpga_pc_reset_n   = Signal()
        self.stream_fifo_fpga_pc_reset_n = Signal()
        self.stream_fifo_pc_fpga_reset_n = Signal()

        # # #

        self.platform = platform

        # Signals
        # -------

        # FT601 TOP.
        # -------------------------

        self.comb += pads.WAKEUPn.eq(1)

        self.specials += Instance("FT601_top",
            # Parameters
            p_FT_data_width      = FT_data_width,
            p_FT_be_width        = FT_be_width,
            p_EP02_rdusedw_width = EP02_rdusedw_width,
            p_EP02_rwidth        = EP02_rwidth,
            p_EP82_wrusedw_width = EP82_wrusedw_width,
            p_EP82_wwidth        = EP82_wwidth,
            p_EP82_wsize         = EP82_wsize,
            p_EP03_rdusedw_width = EP03_rdusedw_width,
            p_EP03_rwidth        = EP03_rwidth,
            p_EP83_wrusedw_width = EP83_wrusedw_width,
            p_EP83_wwidth        = EP83_wwidth,
            p_EP83_wsize         = EP83_wsize,

            # Clk/Reset
            i_clk                = clk_pads,
            i_reset_n            = ~ResetSignal("sys"),

            # FTDI external ports
            o_FT_wr_n            = pads.WRn,
            i_FT_rxf_n           = pads.RXFn,
            io_FT_data           = pads.D,
            io_FT_be             = pads.BE,
            i_FT_txe_n           = pads.TXEn,
            o_FT_RESETn          = pads.RESETn,

            # control endpoint fifo PC->FPGA
            i_EP02_rdclk         = ClockSignal("osc"),
            i_EP02_rd            = self.ctrl_fifo_pc_fpga.valid,
            o_EP02_rdata         = self.ctrl_fifo_pc_fpga.data,
            o_EP02_rempty        = self.ctrl_fifo_pc_fpga.empty,

            # control endpoint fifo FPGA->PC
            i_EP82_wclk          = ClockSignal("osc"),
            i_EP82_aclrn         = self.ctrl_fifo_fpga_pc_reset_n,
            i_EP82_wr            = self.ctrl_fifo_fpga_pc.valid,
            i_EP82_wdata         = self.ctrl_fifo_fpga_pc.data,
            o_EP82_wfull         = self.ctrl_fifo_fpga_pc.full,

            # stream endpoint fifo PC->FPGA
            o_EP03_active        = self.stream_fifo_pc_fpga.active,
            i_EP03_aclrn         = self.stream_fifo_pc_fpga_reset_n,
            i_EP03_rdclk         = ClockSignal("lms_tx"),
            i_EP03_rd            = self.stream_fifo_pc_fpga.valid,
            o_EP03_rdata         = self.stream_fifo_pc_fpga.data,
            o_EP03_rempty        = self.stream_fifo_pc_fpga.empty,
            o_EP03_rusedw        = self.stream_fifo_pc_fpga.usedw,

            # stream endpoint fifo FPGA->PC
            o_EP83_active        = self.stream_fifo_fpga_pc.active,
            i_EP83_wclk          = ClockSignal("lms_rx"),
            i_EP83_aclrn         = self.stream_fifo_fpga_pc_reset_n,
            i_EP83_wr            = self.stream_fifo_fpga_pc.valid,
            i_EP83_wdata         = self.stream_fifo_fpga_pc.data,
            o_EP83_wfull         = self.stream_fifo_fpga_pc.full,
            o_EP83_wrusedw       = self.stream_fifo_fpga_pc.usedw,
        )

        self.add_sources()

    def add_sources(self):
        ft601_files = [
            "LimeSDR-Mini_lms7_trx/src/FT601/synth/FT601_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/FT601/synth/FT601_arb.vhd",
            "LimeSDR-Mini_lms7_trx/src/FT601/synth/FT601.vhd",
        ]

        for file in ft601_files:
            self.platform.add_source(file)
