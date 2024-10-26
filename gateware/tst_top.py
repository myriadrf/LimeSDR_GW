#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# TST Top ------------------------------------------------------------------------------------------

class TstTop(LiteXModule):
    def __init__(self, platform, fx3_clk, lmk_clk, add_csr=True):
        self.test_en           = Signal(6)
        self.test_frc_err      = Signal(6)
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

        self.tx_tst_i          = Signal(16)
        self.tx_tst_q          = Signal(16)

        # # #

        # Tst Clock Test (tst_top is a wrapper with specials records).
        # ------------------------------------------------------------

        self.specials += Instance("clock_test",
            # input ports
            i_FX3_clk            = fx3_clk,
            i_reset_n            = ~ResetSignal("sys"),
            i_test_en            = self.test_en[0:4],
            i_test_frc_err       = self.test_frc_err[0:4],
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

        if add_csr:
            self.add_csr()

    def add_sources(self, platform):
        tst_top_files = [
            "LimeSDR-Mini_lms7_trx/src/self_test/tst_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/self_test/clock_test.vhd",
        ]

        for file in tst_top_files:
            platform.add_source(file)

    def add_csr(self):
        self._test_en          = CSRStorage(fields=[             # 1
            CSRField("fx3_pclk_tst_en", size=1, offset=0),
            CSRField("Si5351C_tst_en",  size=1, offset=1),
            CSRField("vctcxo_tst_en",   size=1, offset=2),
            CSRField("adf_tst_en",      size=1, offset=3),
            CSRField("ddr2_1_tst_en",   size=1, offset=4),
            CSRField("ddr2_2_tst_en",   size=1, offset=5),
        ])

        self._test_frc_err     = CSRStorage(fields=[             # 3
            CSRField("fx3_pclk_tst_frc_err", size=1, offset=0),
            CSRField("Si5351Ck_tst_frc_err", size=1, offset=1),
            CSRField("vctco_tst_frc_err",    size=1, offset=2),
            CSRField("adf_tst_frc_err",      size=1, offset=3),
            CSRField("ddr2_1_tst_frc_err",   size=1, offset=4),
            CSRField("ddr2_2_tst_frc_err",   size=1, offset=5),
        ])
        self._test_cmplt        = CSRStatus(6)                   # 5
        self._test_rez          = CSRStatus(6)                   # 7
        self._fx3_clk_cnt       = CSRStatus(16) # 9
        self._Si5351C_clk_0_cnt = CSRStatus(16) # 10
        self._Si5351C_clk_1_cnt = CSRStatus(16) # 11
        self._Si5351C_clk_2_cnt = CSRStatus(16) # 12
        self._Si5351C_clk_3_cnt = CSRStatus(16) # 13
        self._Si5351C_clk_5_cnt = CSRStatus(16) # 15
        self._Si5351C_clk_6_cnt = CSRStatus(16) # 16
        self._Si5351C_clk_7_cnt = CSRStatus(16) # 17
        self._lmk_clk_cnt0      = CSRStatus(16) # 18
        self._lmk_clk_cnt1      = CSRStatus(16) # 19
        self._adf_cnt           = CSRStatus(16) # 20

        self._tx_tst_i          = CSRStorage(16, reset=0xAAAA) # 29
        self._tx_tst_q          = CSRStorage(16, reset=0x5555) # 30

        self.comb += [
            self.test_en.eq(                  self._test_en.storage),
            self.test_frc_err.eq(             self._test_frc_err.storage),
            self._test_cmplt.status.eq(       self.test_cmplt),
            self._test_rez.status.eq(         self.test_rez),
            self._fx3_clk_cnt.status.eq(      self.fx3_clk_cnt),
            self._Si5351C_clk_0_cnt.status.eq(self.Si5351C_clk_0_cnt),
            self._Si5351C_clk_1_cnt.status.eq(self.Si5351C_clk_1_cnt),
            self._Si5351C_clk_2_cnt.status.eq(self.Si5351C_clk_2_cnt),
            self._Si5351C_clk_3_cnt.status.eq(self.Si5351C_clk_3_cnt),
            self._Si5351C_clk_5_cnt.status.eq(self.Si5351C_clk_5_cnt),
            self._Si5351C_clk_6_cnt.status.eq(self.Si5351C_clk_6_cnt),
            self._Si5351C_clk_6_cnt.status.eq(self.Si5351C_clk_6_cnt),
            self._lmk_clk_cnt0.status.eq(     self.lmk_clk_cnt[0:16]),
            self._lmk_clk_cnt1.status.eq(     Cat(self.lmk_clk_cnt[16:24], Constant(0, 8))),
            self._adf_cnt.status.eq(          self.adf_muxout_cnt),
            self.tx_tst_i.eq(                 self._tx_tst_i.storage),
            self.tx_tst_q.eq(                 self._tx_tst_q.storage),
        ]
