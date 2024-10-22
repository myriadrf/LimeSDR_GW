#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

# TST Top ------------------------------------------------------------------------------------------

class TstTop(LiteXModule):
    def __init__(self, platform, fx3_clk, lmk_clk):
        self.test_en           = Signal(4)
        self.test_frc_err      = Signal(4)
        self.test_cmplt        = Signal(4)
        self.test_rez          = Signal(4)

        self.Si5351C_clk_0     = Signal()
        self.Si5351C_clk_1     = Signal()
        self.Si5351C_clk_2     = Signal()
        self.Si5351C_clk_3     = Signal()
        self.Si5351C_clk_5     = Signal()
        self.Si5351C_clk_6     = Signal()
        self.Si5351C_clk_7     = Signal()
        self.adf_muxout        = Signal()

        self.fx3_clk_cnt       = Signal(16)
        self.Si5351C_clk_0_cnt = Signal(16)
        self.Si5351C_clk_1_cnt = Signal(16)
        self.Si5351C_clk_2_cnt = Signal(16)
        self.Si5351C_clk_3_cnt = Signal(16)
        self.Si5351C_clk_5_cnt = Signal(16)
        self.Si5351C_clk_6_cnt = Signal(16)
        self.Si5351C_clk_7_cnt = Signal(16)
        self.lmk_clk_cnt       = Signal(24)
        self.adf_muxout_cnt    = Signal(16)

        # # #

        # Tst Clock Test (tst_top is a wrapper with specials records).
        # ------------------------------------------------------------

        self.specials += Instance("clock_test",
            # input ports
            i_FX3_clk            = fx3_clk,
            i_reset_n            = ~ResetSignal("sys"),
            i_test_en            = self.test_en,
            i_test_frc_err       = self.test_frc_err,
            o_test_cmplt         = self.test_cmplt,
            o_test_rez           = self.test_rez,

            i_Si5351C_clk_0      = self.Si5351C_clk_0,
            i_Si5351C_clk_1      = self.Si5351C_clk_1,
            i_Si5351C_clk_2      = self.Si5351C_clk_2,
            i_Si5351C_clk_3      = self.Si5351C_clk_3,
            i_Si5351C_clk_5      = self.Si5351C_clk_5,
            i_Si5351C_clk_6      = self.Si5351C_clk_6,
            i_Si5351C_clk_7      = self.Si5351C_clk_7,
            i_LMK_CLK            = lmk_clk,
            i_ADF_MUXOUT         = self.adf_muxout,

            o_FX3_clk_cnt        = self.fx3_clk_cnt,
            o_Si5351C_clk_0_cnt  = self.Si5351C_clk_0_cnt,
            o_Si5351C_clk_1_cnt  = self.Si5351C_clk_1_cnt,
            o_Si5351C_clk_2_cnt  = self.Si5351C_clk_2_cnt,
            o_Si5351C_clk_3_cnt  = self.Si5351C_clk_3_cnt,
            o_Si5351C_clk_5_cnt  = self.Si5351C_clk_5_cnt,
            o_Si5351C_clk_6_cnt  = self.Si5351C_clk_6_cnt,
            o_Si5351C_clk_7_cnt  = self.Si5351C_clk_7_cnt,
            o_LMK_CLK_cnt        = self.lmk_clk_cnt,
            o_ADF_MUXOUT_cnt     = self.adf_muxout_cnt,
        )

        self.add_sources(platform)

    def add_sources(self, platform):
        tst_top_files = [
            "LimeSDR-Mini_lms7_trx/src/self_test/tst_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/self_test/clock_test.vhd",
        ]

        for file in tst_top_files:
            platform.add_source(file)
