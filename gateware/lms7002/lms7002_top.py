#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

import os

from migen import *
from migen.genlib.cdc       import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.vhd2v_converter import VHD2VConverter

from litex.gen import *

from litex.soc.interconnect.csr import *

from gateware.lms7002.lms7002_rxiq import LMS7002RXIQ
from gateware.lms7002.lms7002_txiq import LMS7002TXIQ
from gateware.lms7002.lms7002_clk  import LMS7002CLK

# UTILS --------------------------------------------------------------------------------------------

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

        self.platform  = platform

        self.pads                = pads
        self.periph_output_val_1 = Signal(16)

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

        # Clock Domains.
        # --------------
        self.cd_lms_rx = ClockDomain()
        self.cd_lms_tx = ClockDomain()

        # Signals.
        # --------
        smpl_cmp_done_sync  = Signal()
        smpl_cmp_error_sync = Signal()

        delay_sel           = Signal(2)

        inst0_loadn         = Signal()
        inst0_move          = Signal()

        inst1_delayf_loadn  = Signal()
        inst1_delayf_move   = Signal()

        tx_data_loadn       = Signal()
        tx_data_move        = Signal()

        rx_data_loadn       = Signal()
        rx_data_move        = Signal()

        # CMP Resync.
        # -----------
        self.specials += [
            MultiReg(self.smpl_cmp.error, smpl_cmp_error_sync, "lms_rx"),
            MultiReg(self.smpl_cmp.done,  smpl_cmp_done_sync,  "lms_rx"),
        ]

        # Clocks.
        # -------
        self.lms7002_clk = LMS7002CLK(pads)
        self.comb += [
            self.cd_lms_tx.clk.eq(pads.MCLK1),
            self.cd_lms_rx.clk.eq(pads.MCLK2),
            # Configuration
            self.lms7002_clk.sel.eq(      Constant(0, 1)), # 0 - fclk1 control, 1 - fclk2 control
            self.lms7002_clk.direction.eq(Constant(0, 1)),
            self.lms7002_clk.loadn.eq(    inst0_loadn),
            self.lms7002_clk.move.eq(     inst0_move),
        ]

        # TX Path (DIQ1).
        # ---------------
        self.lms7002_txiq = ClockDomainsRenamer("lms_tx")(LMS7002TXIQ(12, pads))
        self.comb += [
            # Delay control
            self.lms7002_txiq.data_loadn.eq(    tx_data_loadn),
            self.lms7002_txiq.data_move.eq(     tx_data_move),
            self.lms7002_txiq.data_direction.eq(0),

            # From internal logic
            self.lms7002_txiq.tx_diq1_h.eq(      self.tx_diq1_h),
            self.lms7002_txiq.tx_diq1_l.eq(      self.tx_diq1_l),
        ]

        # RX path (DIQ2).
        # ---------------
        self.lms7002_rxiq = ClockDomainsRenamer("lms_rx")(LMS7002RXIQ(12, pads))
        self.comb += [
            self.lms7002_rxiq.data_loadn.eq(    rx_data_loadn),
            self.lms7002_rxiq.data_move.eq(     rx_data_move),
            self.lms7002_rxiq.data_direction.eq(Constant(0, 1)),

            # Output ports to internal logic.
            self.rx_diq2_h.eq(self.lms7002_rxiq.rx_diq2_h),
            self.rx_diq2_l.eq(self.lms7002_rxiq.rx_diq2_l),
        ]

        # Delay control module.
        # ---------------------
        self.delay_ctrl_top_params = dict()

        # Delay Ctrl Top Signals
        self.delay_ctrl_top_params.update(
            # Clk/Reset.
            i_clk              = ClockSignal("lms_rx"),
            i_reset_n          = ~ResetSignal("lms_rx"),

            #
            i_delay_en         = self.delay_ctrl_en,
            i_delay_sel        = self.delay_ctrl_sel,
            i_delay_dir        = self.delay_ctrl_dir,
            i_delay_mode       = self.delay_ctrl_mode,
            o_delay_done       = self.delay_ctrl_done,
            o_delay_error      = self.delay_ctrl_error,

            # signals from sample compare module (required for automatic phase searching)
            o_smpl_cmp_en      = self.smpl_cmp.en,
            i_smpl_cmp_done    = smpl_cmp_done_sync,
            i_smpl_cmp_error   = smpl_cmp_error_sync,
            o_smpl_cmp_cnt     = self.smpl_cmp.cnt,

            o_delayf_loadn     = inst1_delayf_loadn,
            o_delayf_move      = inst1_delayf_move,
            o_delayf_direction = Open(),
        )

        # Logic.
        # ------
        self.comb += [
            # lms7002_clk"
            If(~self.delay_ctrl_sel == 0b00,
                inst0_loadn.eq(inst1_delayf_loadn),
                inst0_move.eq (inst1_delayf_move),
            ).Else(
                inst0_loadn.eq(1),
                inst0_move.eq (0),
            ),
            # lms7002_tx
            If(self.delay_ctrl_sel == 0b01,
                tx_data_loadn.eq(inst1_delayf_loadn),
                tx_data_move.eq (inst1_delayf_move),
            ).Else(
                tx_data_loadn.eq(1),
                tx_data_move.eq (0),
            ),
            # lms7002_rx
            If(self.delay_ctrl_sel == 0b11,
                rx_data_loadn.eq(inst1_delayf_loadn),
                rx_data_move.eq (inst1_delayf_move),
            ).Else(
                rx_data_loadn.eq(1),
                rx_data_move.eq (0),
            ),
        ]

        self.specials += AsyncResetSynchronizer(self.cd_lms_rx, ResetSignal("sys"))
        self.specials += AsyncResetSynchronizer(self.cd_lms_tx, ResetSignal("sys"))

        if add_csr:
            self.add_csr()

    def add_csr(self):
        # LMS Ctrl GPIO
        self._lms_ctr_gpio = CSRStorage(size=4, description="LMS Control GPIOs.")

        # fpgacfg
        self.lms1              = CSRStorage(fields=[         # 19
            CSRField("ss",          size=1, offset=0, reset=1),
            CSRField("reset",       size=1, offset=1, reset=1),
            CSRField("core_ldo_en", size=1, offset=2, reset=0),
            CSRField("txnrx1",      size=1, offset=3, reset=1),
            CSRField("txnrx2",      size=1, offset=4, reset=0),
            CSRField("txen",        size=1, offset=5, reset=1),
            CSRField("rxen",        size=1, offset=6, reset=1),
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
                self.pads.TXNRX2_or_CLK_SEL.eq(self.periph_output_val_1),
            ).Else(
                self.pads.TXNRX2_or_CLK_SEL.eq(self.lms1.fields.txnrx2),
            ),
            self.pads.TXEN.eq(       self.lms1.fields.txen),
            self.pads.RXEN.eq(       self.lms1.fields.rxen),
            self.pads.CORE_LDO_EN.eq(self.lms1.fields.core_ldo_en),
            self.pads.TXNRX1.eq(     self.lms1.fields.txnrx1),
            self.pads.RESET.eq(      self.lms1.fields.reset & self._lms_ctr_gpio.storage[0]),

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

    def do_finalize(self):
        self.delay_ctrl_top = VHD2VConverter(self.platform,
            top_entity    = "delay_ctrl_top",
            build_dir     = os.path.abspath(self.platform.output_dir),
            work_package  = "work",
            force_convert = True,
            params        = self.delay_ctrl_top_params,
            add_instance  = True,
        )
        self.delay_ctrl_top.add_source("gateware/hdl/delayf_ctrl/delay_ctrl_fsm.vhd")
        self.delay_ctrl_top.add_source("gateware/hdl/delayf_ctrl/delay_ctrl_top.vhd")
        self.delay_ctrl_top.add_source("gateware/hdl/delayf_ctrl/delayf_ctrl.vhd")

