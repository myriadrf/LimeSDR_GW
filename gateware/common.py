#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

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
    ("ch_en",             16),
    ("smpl_width",         2),
    ("mode",               1),
    ("ddr_en",             1),
    ("trxiq_pulse",        1),
    ("mimo_int_en",        1),
    ("synch_dis",          1),
    ("synch_mode",         1),
    ("smpl_nr_clr",        1),
    ("txpct_loss_clr",     1),
    ("rx_en",              1),
    ("tx_en",              1),
    ("rx_ptrn_en",         1),
    ("tx_ptrn_en",         1),
    ("tx_cnt_en",          1),
    ("wfm_ch_en",         16),
    ("wfm_play",           1),
    ("wfm_load",           1),
    ("wfm_smpl_width",     2),
    ("SPI_SS",            16),
    ("LMS1_SS",            1),
    ("LMS1_RESET",         1),
    ("LMS1_CORE_LDO_EN",   1),
    ("LMS1_TXNRX1",        1),
    ("LMS1_TXNRX2",        1),
    ("LMS1_TXEN",          1),
    ("LMS1_RXEN",          1),
    ("GPIO",              16),
    ("FPGA_LED1_CTRL",     3),
    ("FPGA_LED2_CTRL",     3),
    ("FX3_LED_CTRL",       3),
    ("CLK_ENA",            4),
    ("sync_pulse_period", 32),
    ("sync_size",         16),
    ("txant_pre",         16),
    ("txant_post",        16),
]

to_tstcfg_from_rxtx_layout = [
    ("DDR2_1_STATUS",       3),
    ("DDR2_1_pnf_per_bit", 32),
]

from_tstcfg_layout = [
    ("TEST_EN",      6),
    ("TEST_FRC_ERR", 6),
    ("TX_TST_I",    16),
    ("TX_TST_Q",    16),
]

def FromFPGACfg():
    return Record(from_fpgacfg_layout)

def ToTstCfgFromRXTX():
    return Record(to_tstcfg_from_rxtx_layout)

def FromTstCfg():
    return Record(from_tstcfg_layout)
