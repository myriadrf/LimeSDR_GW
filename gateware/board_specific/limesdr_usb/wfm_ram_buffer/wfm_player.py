#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

# WFM Player ---------------------------------------------------------------------------------------

class WFMPlayer(LiteXModule):
    def __init__(self, platform,
        dev_family       = "Cyclone IV E",
        wfm_infifo_size  = 11,
        wfm_outfifo_size = 11,
        data_width       = 32,
        iq_width         = 12,
        addr_size        = 24,
        cntrl_bus_size   = 16,
        lcl_burst_length = 2,
        cntrl_rate       = 1
    ):
        self.platform = platform

        # # #

        self.specials += Instance("wfm_player",
            # Parameters
            p_dev_family       = dev_family,
            p_wfm_infifo_size  = wfm_infifo_size,
            p_wfm_outfifo_size = wfm_outfifo_size,
            p_data_width       = data_width,
            p_iq_width         = iq_width,
            p_addr_size        = addr_size,
            p_cntrl_bus_size   = cntrl_bus_size,
            p_lcl_burst_length = lcl_burst_length,
            p_cntrl_rate       = cntrl_rate,

            # Inputs
            i_ddr2_phy_clk         = Constant(0),
            i_ddr2_phy_reset_n     = Constant(0),
            i_wfm_load             = Constant(0),
            i_wfm_play_stop        = Constant(0),
            i_wfm_infifo_data      = Constant(0, data_width),
            i_wfm_infifo_rdempty   = Constant(0),
            i_wfm_infifo_rdusedw   = Constant(0, wfm_infifo_size),
            i_wcmd_clk             = Constant(0),
            i_wcmd_reset_n         = Constant(0),
            i_wcmd_rdy             = Constant(0),
            i_rcmd_clk             = Constant(0),
            i_rcmd_reset_n         = Constant(0),
            i_rcmd_rdy             = Constant(0),

            # Outputs
            o_wfm_infifo_reset_n_req = Open(),
            o_wfm_infifo_rdreq       = Open(),
            o_wcmd_addr              = Open(),
            o_wcmd_wr                = Open(),
            o_wcmd_brst_en           = Open(),
            o_wcmd_data              = Open(),
            o_rcmd_addr              = Open(),
            o_rcmd_wr                = Open(),
            o_rcmd_brst_en           = Open(),
        )

        self.add_sources(platform)

    def add_sources(self, platform):
        base_path = "gateware/board_specific/limesdr_usb/wfm_ram_buffer/"
        sources = [
            "wfm_player.vhd",
            "wfm_wcmd_fsm.vhd",
            "wfm_rcmd_fsm.vhd",
            # "fifo_inst.vhd", # Placeholder: This file is declared but not found in the directory.
        ]
        for s in sources:
            platform.add_source(base_path + s)

        # Note: fifo_inst.vhd is required by wfm_player.vhd but not found in this directory.
        # platform.add_source(base_path + "fifo_inst.vhd")
