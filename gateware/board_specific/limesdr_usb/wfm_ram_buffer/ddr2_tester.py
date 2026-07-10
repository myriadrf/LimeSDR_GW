#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

# DDR2 Tester (LimeSDR-USB) ------------------------------------------------------------------------------

class DDR2Tester(LiteXModule):
    def __init__(self, platform):
        self.platform = platform

        # # #

        self.specials += Instance("ddr2_tester",
            # Inputs
            i_global_reset_n    = Constant(0),
            i_pll_ref_clk       = Constant(0),
            i_soft_reset_n      = Constant(0),
            i_begin_test        = Constant(0),
            i_insert_error      = Constant(0),

            # Outputs
            o_mem_odt           = Open(),
            o_mem_cs_n          = Open(),
            o_mem_cke           = Open(),
            o_mem_addr          = Open(),
            o_mem_ba            = Open(),
            o_mem_ras_n         = Open(),
            o_mem_cas_n         = Open(),
            o_mem_we_n          = Open(),
            o_mem_dm            = Open(),
            o_mem_clk           = Open(), # inout
            o_mem_clk_n         = Open(), # inout
            o_mem_dq            = Open(), # inout
            o_mem_dqs           = Open(), # inout

            o_pnf_per_bit         = Open(),
            o_pnf_per_bit_persist = Open(),
            o_pass                = Open(),
            o_fail                = Open(),
            o_test_complete       = Open(),
        )

        self.add_sources(platform)

    def add_sources_ddr_test(self, platform):
        ddr2_tester_files = [
            "gateware/board_specific/limesdr_usb/wfm_ram_buffer/ddr2_tester.vhd"
        ]

        for file in ddr2_tester_files:
            platform.add_source(file)

        ddr2_tester_ips = [
            "gateware/board_specific/limesdr_usb/wfm_ram_buffer/ddr2_traffic_gen.qsys",
            "gateware/board_specific/limesdr_usb/wfm_ram_buffer/ddr2.qip",
        ]

        for ip in ddr2_tester_ips:
            platform.add_ip(ip)
