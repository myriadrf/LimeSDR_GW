#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from gateware.common            import add_vhd2v_converter

# TST Top ------------------------------------------------------------------------------------------

class TstTop(LiteXModule):
    def __init__(self, platform, fx3_clk, lmk_clk, add_csr=True):

        self.platform          = platform

        self.test_en           = Signal(6)
        self.test_frc_err      = Signal(6)
        self.test_cmplt        = Signal(4)
        self.test_rez          = Signal(4)

        self.adf_muxout        = Signal()

        self.fx3_clk_cnt       = Signal(16)
        self.lmk_clk_cnt       = Signal(24)
        self.adf_muxout_cnt    = Signal(16)

        self.tx_tst_i          = Signal(16)
        self.tx_tst_q          = Signal(16)

        # # #

        # Tst Clock Test (tst_top is a wrapper with specials records).
        # ------------------------------------------------------------

        self.clock_test_params = dict()
        self.clock_test_params.update(
            # input ports
            i_FX3_clk            = fx3_clk,
            i_reset_n            = ~ResetSignal("sys"),
            i_test_en            = self.test_en[0:4],
            i_test_frc_err       = self.test_frc_err[0:4],
            o_test_cmplt         = self.test_cmplt,
            o_test_rez           = self.test_rez,

            i_Si5351C_clk_0      = 0b0,
            i_Si5351C_clk_1      = 0b0,
            i_Si5351C_clk_2      = 0b0,
            i_Si5351C_clk_3      = 0b0,
            i_Si5351C_clk_5      = 0b0,
            i_Si5351C_clk_6      = 0b0,
            i_Si5351C_clk_7      = 0b0,
            i_LMK_CLK            = lmk_clk,
            i_ADF_MUXOUT         = self.adf_muxout,

            o_FX3_clk_cnt        = self.fx3_clk_cnt,
            o_Si5351C_clk_0_cnt  = Open(16),
            o_Si5351C_clk_1_cnt  = Open(16),
            o_Si5351C_clk_2_cnt  = Open(16),
            o_Si5351C_clk_3_cnt  = Open(16),
            o_Si5351C_clk_5_cnt  = Open(16),
            o_Si5351C_clk_6_cnt  = Open(16),
            o_Si5351C_clk_7_cnt  = Open(16),
            o_LMK_CLK_cnt        = self.lmk_clk_cnt,
            o_ADF_MUXOUT_cnt     = self.adf_muxout_cnt,
        )

        if add_csr:
            self.add_csr()

    def add_csr(self):
        self._test_en          = CSRStorage(fields=[
            CSRField("fx3_pclk_tst_en", size=1, offset=0),
            CSRField("vctcxo_tst_en",   size=1, offset=2),
            CSRField("adf_tst_en",      size=1, offset=3),
            CSRField("ddr2_1_tst_en",   size=1, offset=4),
            CSRField("ddr2_2_tst_en",   size=1, offset=5),
        ])

        self._test_frc_err     = CSRStorage(fields=[
            CSRField("fx3_pclk_tst_frc_err", size=1, offset=0),
            CSRField("vctco_tst_frc_err",    size=1, offset=2),
            CSRField("adf_tst_frc_err",      size=1, offset=3),
            CSRField("ddr2_1_tst_frc_err",   size=1, offset=4),
            CSRField("ddr2_2_tst_frc_err",   size=1, offset=5),
        ])
        self._test_cmplt        = CSRStatus(6)
        self._test_rez          = CSRStatus(6)
        self._fx3_clk_cnt       = CSRStatus(16)
        self._lmk_clk_cnt0      = CSRStatus(16)
        self._lmk_clk_cnt1      = CSRStatus(16)
        self._adf_cnt           = CSRStatus(16)

        self._tx_tst_i          = CSRStorage(16, reset=0xAAAA)
        self._tx_tst_q          = CSRStorage(16, reset=0x5555)

        self.comb += [
            self.test_en.eq(                  self._test_en.storage),
            self.test_frc_err.eq(             self._test_frc_err.storage),
            self._test_cmplt.status.eq(       self.test_cmplt),
            self._test_rez.status.eq(         self.test_rez),
            self._fx3_clk_cnt.status.eq(      self.fx3_clk_cnt),
            self._lmk_clk_cnt0.status.eq(     self.lmk_clk_cnt[0:16]),
            self._lmk_clk_cnt1.status.eq(     Cat(self.lmk_clk_cnt[16:24], Constant(0, 8))),
            self._adf_cnt.status.eq(          self.adf_muxout_cnt),
            self.tx_tst_i.eq(                 self._tx_tst_i.storage),
            self.tx_tst_q.eq(                 self._tx_tst_q.storage),
        ]

    def do_finalize(self):
        clock_test_files = [
            "gateware/hdl/self_test/transition_count.vhd",
            "gateware/hdl/self_test/singl_clk_with_ref_test.vhd",
            "gateware/hdl/self_test/clk_with_ref_test.vhd",
            "gateware/hdl/self_test/clk_no_ref_test.vhd",
            "gateware/hdl/self_test/clock_test.vhd",
        ]

        self.clock_test = add_vhd2v_converter(self.platform,
            top    = "clock_test",
            params = self.clock_test_params,
            files  = clock_test_files,
        )
