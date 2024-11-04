#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

class FIFOInterface(LiteXModule):
    def __init__(self, rdata_width, wdata_wdith, rdusedw_size=1, wrusedw_size=1):
        self.rd          = Signal()
        self.wr          = Signal()
        self.rd_active   = Signal()
        self.wr_active   = Signal()
        self.rdata       = Signal(rdata_width)
        self.wdata       = Signal(wdata_wdith)
        self.empty       = Signal()
        self.full        = Signal()
        self.rdusedw     = Signal(rdusedw_size)
        self.wrusedw     = Signal(wrusedw_size)

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

# Layout -------------------------------------------------------------------------------------------

from_fpgacfg_layout = [
    ("phase_reg_sel", 16),       # std_logic_vector(15 downto 0)
    ("clk_ind", 5),              # std_logic_vector(4 downto 0)
    ("cnt_ind", 5),              # std_logic_vector(4 downto 0)
    ("load_phase_reg", 1),       # std_logic
    ("drct_clk_en", 16),         # std_logic_vector(15 downto 0)
    ("ch_en", 16),               # std_logic_vector(15 downto 0)
    ("smpl_width", 2),           # std_logic_vector(1 downto 0)
    ("mode", 1),                 # std_logic
    ("ddr_en", 1),               # std_logic
    ("trxiq_pulse", 1),          # std_logic
    ("mimo_int_en", 1),          # std_logic
    ("synch_dis", 1),            # std_logic
    ("synch_mode", 1),           # std_logic
    ("smpl_nr_clr", 1),          # std_logic
    ("txpct_loss_clr", 1),       # std_logic
    ("rx_en", 1),                # std_logic
    ("tx_en", 1),                # std_logic
    ("rx_ptrn_en", 1),           # std_logic
    ("tx_ptrn_en", 1),           # std_logic
    ("tx_cnt_en", 1),            # std_logic
    ("wfm_ch_en", 16),           # std_logic_vector(15 downto 0)
    ("wfm_play", 1),             # std_logic
    ("wfm_load", 1),             # std_logic
    ("wfm_smpl_width", 2),       # std_logic_vector(1 downto 0)
    ("SPI_SS", 16),              # std_logic_vector(15 downto 0)
    ("LMS1_SS", 1),              # std_logic
    ("LMS1_RESET", 1),           # std_logic
    ("LMS1_CORE_LDO_EN", 1),     # std_logic
    ("LMS1_TXNRX1", 1),          # std_logic
    ("LMS1_TXNRX2", 1),          # std_logic
    ("LMS1_TXEN", 1),            # std_logic
    ("LMS1_RXEN", 1),            # std_logic
    ("GPIO", 16),                # std_logic_vector(15 downto 0)
    ("FPGA_LED1_CTRL", 3),       # std_logic_vector(2 downto 0)
    ("FPGA_LED2_CTRL", 3),       # std_logic_vector(2 downto 0)
    ("FX3_LED_CTRL", 3),         # std_logic_vector(2 downto 0)
    ("CLK_ENA", 4),              # std_logic_vector(3 downto 0)
    ("sync_pulse_period", 32),   # std_logic_vector(31 downto 0)
    ("sync_size", 16),           # std_logic_vector(15 downto 0)
    ("txant_pre", 16),           # std_logic_vector(15 downto 0)
    ("txant_post", 16),          # std_logic_vector(15 downto 0)
]

to_tstcfg_from_rxtx_layout = [
    ("DDR2_1_STATUS", 3),           # std_logic_vector(2 downto 0)
    ("DDR2_1_pnf_per_bit", 32),     # std_logic_vector(31 downto 0)
]

from_tstcfg_layout = [
    ("TEST_EN", 6),      # std_logic_vector(5 downto 0);
    ("TEST_FRC_ERR", 6), # std_logic_vector(5 downto 0);
    ("TX_TST_I", 16),    # std_logic_vector(15 downto 0);
    ("TX_TST_Q", 16),    # std_logic_vector(15 downto 0);
]

def FromFPGACfg():
    return Record(from_fpgacfg_layout)

def ToTstCfgFromRXTX():
    return Record(to_tstcfg_from_rxtx_layout)

def FromTstCfg():
    return Record(from_tstcfg_layout)
