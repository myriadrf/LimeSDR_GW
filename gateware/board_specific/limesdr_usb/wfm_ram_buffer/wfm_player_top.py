#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

# WFM Player Top -----------------------------------------------------------------------------------

class WFMPlayerTop(LiteXModule):
    """
    WFM Player Top wrapper for LiteX.

    Parameters
    ----------
    platform : platform
        LiteX platform object.
    ddr_pads : Record
        DDR2 memory pads from the platform.
    dev_family : str
        FPGA device family (e.g., "Cyclone IV E").
    cntrl_rate : int
        DDR2 controller rate (1 for full rate, 2 for half rate).
    cntrl_bus_size : int
        DDR2 controller bus size.
    addr_size : int
        DDR2 address size.
    lcl_bus_size : int
        Local bus size.
    lcl_burst_length : int
        Local burst length.
    cmd_fifo_size : int
        Command FIFO size.
    wfm_infifo_size : int
        WFM input FIFO size.
    wfm_outfifo_size : int
        WFM output FIFO size.
    data_width : int
        Data width.
    iq_width : int
        IQ width.
    dcmpr_fifo_size : int
        Decompression FIFO size.
    """
    def __init__(self, platform, ddr_pads,
        dev_family       = "Cyclone IV E",
        cntrl_rate       = 1,
        cntrl_bus_size   = 16,
        addr_size        = 25,
        lcl_bus_size     = 64,
        lcl_burst_length = 2,
        cmd_fifo_size    = 9,
        wfm_infifo_size  = 11,
        wfm_outfifo_size = 11,
        data_width       = 32,
        iq_width         = 12,
        dcmpr_fifo_size  = 10
    ):
        self.platform           = platform
        self.phy_clk            = Signal()
        self.wfm_load           = Signal()
        self.wfm_play           = Signal()
        self.wfm_smpl_width     = Signal(2)
        self.wfm_ch_en          = Signal(2)
        self.begin_test         = Signal()
        self.insert_error       = Signal()
        self.wfm_infifo_reset_n = Signal() # EXTERNAL CONNECTION NEEDED

        self.diq_l_full   = Signal(16)
        self.diq_h_full   = Signal(16)
        self.diq_l        = Signal(iq_width+1)
        self.diq_h        = Signal(iq_width+1)
        self.pnf_per_bit  = Signal(32)
        self.tst_pass     = Signal()
        self.tst_fail     = Signal()
        self.tst_complete = Signal()
        # # #

        self.specials += Instance("wfm_player_top",
            # Parameters
            p_dev_family       = dev_family,
            p_cntrl_rate       = cntrl_rate,
            p_cntrl_bus_size   = cntrl_bus_size,
            p_addr_size        = addr_size,
            p_lcl_bus_size     = lcl_bus_size,
            p_lcl_burst_length = lcl_burst_length,
            p_cmd_fifo_size    = cmd_fifo_size,
            p_wfm_infifo_size  = wfm_infifo_size,
            p_wfm_outfifo_size = wfm_outfifo_size,
            p_data_width       = data_width,
            p_iq_width         = iq_width,
            p_dcmpr_fifo_size  = dcmpr_fifo_size,

            # Inputs
            i_reset_n               = ~ResetSignal("sys"),
            i_ddr2_pll_ref_clk      = ClockSignal("sys"),
            i_wcmd_clk              = ClockSignal("lms_tx"),
            i_rcmd_clk              = self.phy_clk,
            i_wfm_load              = self.wfm_load,
            i_wfm_play_stop         = self.wfm_play,
            i_wfm_infifo_data       = Constant(0, data_width),      #DO LATER
            i_wfm_infifo_rdempty    = Constant(0),#DO LATER
            i_wfm_infifo_rdusedw    = Constant(0, wfm_infifo_size),#DO LATER
            i_sample_width          = self.wfm_smpl_width,
            i_fr_start              = Constant(0),
            i_ch_en                 = self.wfm_ch_en,
            i_mimo_en               = Constant(1),
            i_iq_clk                = ClockSignal("lms_tx"),
            i_begin_test            = self.begin_test,
            i_insert_error          = self.insert_error,

            # Outputs
            o_wfm_infifo_reset_n_req = self.wfm_infifo_reset_n,
            o_wfm_infifo_rdreq       = Open(),#DO LATER
            o_wfm_rdy                = Open(),
            o_dd_iq_h                = self.diq_h_full,
            o_dd_iq_l                = self.diq_l_full,
            o_mem_odt                = ddr_pads.odt,
            o_mem_cs_n               = ddr_pads.cs_n,
            o_mem_cke                = ddr_pads.cke,
            o_mem_addr               = ddr_pads.a,
            o_mem_ba                 = ddr_pads.ba,
            o_mem_ras_n              = ddr_pads.ras_n,
            o_mem_cas_n              = ddr_pads.cas_n,
            o_mem_we_n               = ddr_pads.we_n,
            o_mem_dm                 = ddr_pads.dm,
            o_phy_clk                = self.phy_clk,
            o_pnf_per_bit            = Open(),
            o_pnf_per_bit_persist    = self.pnf_per_bit,
            o_pass                   = self.tst_pass,
            o_fail                   = self.tst_fail,
            o_test_complete          = self.tst_complete,

            # Inouts
            io_mem_clk               = ddr_pads.clk,
            io_mem_clk_n             = ddr_pads.clk_n,
            io_mem_dq                = ddr_pads.dq,
            io_mem_dqs               = ddr_pads.dqs,
        )

        self.comb += [
            self.diq_h.eq(self.diq_h_full[0:14]),
            self.diq_l.eq(self.diq_l_full[0:14]),
        ]


    # def add_sources(self, platform):
    #     base_path = "gateware/board_specific/limesdr_usb/wfm_ram_buffer/"
    #     sources = [
    #         "wfm_player_top.vhd",
    #         "wfm_player.vhd",
    #         "wfm_wcmd_fsm.vhd",
    #         "wfm_rcmd_fsm.vhd",
    #         "DDR2_ctrl_top.vhd",
    #         "DDR2_arb.vhd",
    #     ]
    #     for s in sources:
    #         platform.add_source(base_path + s)
    #
    #     # IP Cores
    #     platform.add_ip("gateware/board_specific/limesdr_usb/ddr2_traffic_gen/ddr2_traffic_gen.qsys")
    #     platform.add_ip("gateware/board_specific/limesdr_usb/ddr2/ddr2.qip")

        # Note: The following files are referenced in wfm_player_top.vhd but not found in this directory:
        # - decompress.vhd
        # - rd_tx_fifo.vhd
        # - fifo_inst.vhd (referenced in wfm_player.vhd and DDR2_ctrl_top.vhd)
