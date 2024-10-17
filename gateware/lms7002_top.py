#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

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
    def __init__(self, platform, pads=None):

        assert pads is not None

        self.reset_n = Signal()
        self.tx_diq1_h = Signal(13)
        self.tx_diq1_l = Signal(13)

        self.rx_diq2_h = Signal(13)
        self.rx_diq2_l = Signal(13)

        self.delay_control = DelayControl()
        self.smpl_cmp      = SampleCompare()

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
            i_reset_n        = self.reset_n,

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
            i_delay_en       = self.delay_control.en,
            i_delay_sel      = self.delay_control.sel,  # 0 FCLK1, 1 - TX_DIQ(not supported), 2 - FLCK2(not supported), 3 - RX_DIQ
            i_delay_dir      = self.delay_control.dir,
            i_delay_mode     = self.delay_control.mode, # 0 - manual, 1 - auto
            o_delay_done     = self.delay_control.done,
            o_delay_error    = self.delay_control.error,
            # signals from sample compare module (required for automatic phase searching)
            o_smpl_cmp_en    = self.smpl_cmp.en,
            i_smpl_cmp_done  = self.smpl_cmp.done,
            i_smpl_cmp_error = self.smpl_cmp.error,
            o_smpl_cmp_cnt   = self.smpl_cmp.cnt,
        )

        self.add_sources(platform)

    def add_sources(self, platform):
        lms7002_files = [
            "LimeSDR-Mini_lms7_trx/src/lms7002/lms7002_clk.vhd",
            "LimeSDR-Mini_lms7_trx/src/lms7002/lms7002_top.vhd",
            "LimeSDR-Mini_lms7_trx/src/lms7002/lms7002_rxiq.vhd",
            "LimeSDR-Mini_lms7_trx/src/lms7002/lms7002_txiq.vhd",
        ]

        for file in lms7002_files:
            platform.add_source(file)
