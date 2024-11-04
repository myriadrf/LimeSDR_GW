#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# UTILS --------------------------------------------------------------------------------------------

class DelayControl(LiteXModule):
    def __init__(self):
        self.en    = Signal()
        self.sel   = Signal(2)
        self.dir   = Signal()
        self.mode  = Signal()
        self.done  = Signal()
        self.error = Signal()

    def connect(self, slave_delay_control):
        return [
            slave_delay_control.en.eq(self.en),
            slave_delay_control.sel.eq(self.sel),
            slave_delay_control.dir.eq(self.dir),
            slave_delay_control.mode.eq(self.mode),
            self.done.eq(slave_delay_control.done),
            self.error.eq(slave_delay_control.error),
        ]

class SampleCompare(LiteXModule):
    def __init__(self):
        self.en    = Signal()
        self.done  = Signal()
        self.error = Signal()
        self.cnt   = Signal(16)

    def connect(self, slave_sample_compare):
        return [
            slave_sample_compare.en.eq(self.en),
            self.done.eq(slave_sample_compare.done),
            self.error.eq(slave_sample_compare.error),
            slave_sample_compare.cnt.eq(self.cnt),
        ]

# LMS7002 Top --------------------------------------------------------------------------------------

class LMS7002Top(LiteXModule):
    def __init__(self, platform, pads=None, hw_ver=None, add_csr=True):

        assert pads   is not None
        assert hw_ver is not None

        self.pads                = pads
        self.PERIPH_OUTPUT_VAL_1 = Signal(16)

        self.tx_diq1_h = Signal(13)
        self.tx_diq1_l = Signal(13)

        self.rx_diq2_h = Signal(13)
        self.rx_diq2_l = Signal(13)

        self.delay_ctrl_en    = Signal()
        self.delay_ctrl_sel   = Signal(2)
        self.delay_ctrl_dir   = Signal()
        self.delay_ctrl_mode  = Signal()
        self.delay_ctrl_done  = Signal()
        self.delay_ctrl_error = Signal()
        self.smpl_cmp         = SampleCompare()
        self.hw_ver           = Signal(4)

        # # #

        # Clocks.
        # -------
        self.cd_lms_rx = ClockDomain()
        self.cd_lms_tx = ClockDomain()

        # FT601 TOP.
        # -------------------------

        self.specials += Instance("lms7002_top",
            # Free running clock and reset
            i_clk            = self.cd_lms_rx.clk,
            i_reset_n        = ~ResetSignal("sys"),

            # TX DIQ
            i_MCLK1          = pads.MCLK1,
            o_FLCK1          = pads.FCLK1,
            o_ENABLE_IQSEL1  = pads.ENABLE_IQSEL1,
            o_DIQ1_D         = pads.DIQ1_D,

            # RX DIQ
            i_MCLK2          = pads.MCLK2,
            o_FLCK2          = pads.FCLK2,
            i_ENABLE_IQSEL2  = pads.ENABLE_IQSEL2,
            i_DIQ2_D         = pads.DIQ2_D,

            # Internal logic

            # tx
            o_tx_clk         = self.cd_lms_tx.clk,
            i_tx_diq1_h      = self.tx_diq1_h,
            i_tx_diq1_l      = self.tx_diq1_l,

            # rx
            o_rx_clk         = self.cd_lms_rx.clk,
            o_rx_diq2_h      = self.rx_diq2_h,
            o_rx_diq2_l      = self.rx_diq2_l,

            # delay control
            i_delay_en       = self.delay_ctrl_en,
            i_delay_sel      = self.delay_ctrl_sel,  # 0 FCLK1, 1 - TX_DIQ(not supported), 2 - FLCK2(not supported), 3 - RX_DIQ
            i_delay_dir      = self.delay_ctrl_dir,
            i_delay_mode     = self.delay_ctrl_mode, # 0 - manual, 1 - auto
            o_delay_done     = self.delay_ctrl_done,
            o_delay_error    = self.delay_ctrl_error,
            # signals from sample compare module (required for automatic phase searching)
            o_smpl_cmp_en    = self.smpl_cmp.en,
            i_smpl_cmp_done  = self.smpl_cmp.done,
            i_smpl_cmp_error = self.smpl_cmp.error,
            o_smpl_cmp_cnt   = self.smpl_cmp.cnt,
        )

        if add_csr:
            self.add_csr()
        self.add_sources(platform)

    def add_csr(self):
        # LMS Ctrl GPIO
        self._lms_ctr_gpio = CSRStorage(size=4, description="LMS Control GPIOs.")

        self.LMS1              = CSRStorage(fields=[         # 19
            CSRField("SS",          size=1, offset=0, reset=1),
            CSRField("RESET",       size=1, offset=1, reset=1),
            CSRField("CORE_LDO_EN", size=1, offset=2, reset=0),
            CSRField("TXNRX1",      size=1, offset=3, reset=1),
            CSRField("TXNRX2",      size=1, offset=4, reset=0),
            CSRField("TXEN",        size=1, offset=5, reset=1),
            CSRField("RXEN",        size=1, offset=6, reset=1),
        ])

        # pllcfg
        self.reg01     = CSRStatus(16,  reset=1)
        self.reg03     = CSRStorage(16, fields=[
            CSRField("pllcfg_start", size=1, offset=0),
            CSRField("phcfg_start",  size=1, offset=1),
            CSRField("pllrst_start", size=1, offset=2),
            CSRField("pll_ind",      size=5, offset=3),
            CSRField("cnt_ind",      size=5, offset=8),
            CSRField("phcfg_updn",   size=1, offset=13),
            CSRField("phcfg_mode",   size=1, offset=14),
            CSRField("phcfg_tst",    size=1, offset=15),
        ], reset=0)

        self.comb += [
            # LMS Controls.
            If((self.hw_ver > 5),
                self.pads.TXNRX2_or_CLK_SEL.eq(self.PERIPH_OUTPUT_VAL_1),
            ).Else(
                self.pads.TXNRX2_or_CLK_SEL.eq(self.LMS1.fields.TXNRX2),
            ),
            self.pads.TXEN.eq(       self.LMS1.fields.TXEN),
            self.pads.RXEN.eq(       self.LMS1.fields.RXEN),
            self.pads.CORE_LDO_EN.eq(self.LMS1.fields.CORE_LDO_EN),
            self.pads.TXNRX1.eq(     self.LMS1.fields.TXNRX1),
            self.pads.RESET.eq(      self.LMS1.fields.RESET & self._lms_ctr_gpio.storage[0]),

            # pllcfg
            self.reg01.status.eq(Cat(1, 0, self.delay_ctrl_done, self.delay_ctrl_error, Constant(0, 12))),
            self.delay_ctrl_en.eq(self.reg03.fields.phcfg_start),
            If(self.reg03.fields.cnt_ind == 0b0011,
                self.delay_ctrl_sel.eq(0),
            ).Else(
                self.delay_ctrl_sel.eq(3),
            ),
            self.delay_ctrl_dir.eq( self.reg03.fields.phcfg_updn),
            self.delay_ctrl_mode.eq(self.reg03.fields.phcfg_mode),

        ]

    def add_sources(self, platform):
        lms7002_files = [
            "LimeSDR-Mini_lms7_trx/src/lms7002/lms7002_clk.vhd",
            "LimeSDR-Mini_lms7_trx/src/lms7002/lms7002_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/lms7002/lms7002_rxiq.vhd",
            "LimeSDR-Mini_lms7_trx/src/lms7002/lms7002_txiq.vhd",
        ]

        for file in lms7002_files:
            platform.add_source(file)
