#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os

from migen import *

from litex.build.vhd2v_converter import VHD2VConverter

from litex.gen import *

# FIFO Interface -----------------------------------------------------------------------------------

class FIFOInterface(LiteXModule):
    def __init__(self, rdata_width, wdata_wdith, rdusedw_size=1, wrusedw_size=1):
        self.rd        = Signal()
        self.wr        = Signal()
        self.rd_active = Signal()
        self.wr_active = Signal()
        self.rdata     = Signal(rdata_width)
        self.wdata     = Signal(wdata_wdith)
        self.empty     = Signal()
        self.full      = Signal()
        self.rdusedw   = Signal(rdusedw_size)
        self.wrusedw   = Signal(wrusedw_size)

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

# Configuration Layouts ----------------------------------------------------------------------------

from_fpgacfg_layout = [
    ("phase_reg_sel",     16),
    ("clk_ind",            5),
    ("cnt_ind",            5),
    ("load_phase_reg",     1),
    ("drct_clk_en",       16),
    ("wfm_ch_en",         16),
    ("wfm_smpl_width",     2),
    ("spi_ss",            16),
    ("gpio",              16),
    ("fpga_led1_ctrl",     3),
    ("fpga_led2_ctrl",     3),
    ("fx3_led_ctrl",       3),
    ("clk_ena",            4),
    ("sync_pulse_period", 32),
]

def FromFPGACfg():
    return Record(from_fpgacfg_layout)

# VHD2VConverter Wrapper ---------------------------------------------------------------------------
def add_vhd2v_converter(platform, instance, files=[], force_convert=None):
    force_convert = {True: LiteXContext.platform.vhd2v_force, False: force_convert}[force_convert is None]
    return VHD2VConverter(platform,
        instance      = instance,
        work_package  = "work",
        force_convert = force_convert,
        add_instance  = True,
        files         = files,
    )
