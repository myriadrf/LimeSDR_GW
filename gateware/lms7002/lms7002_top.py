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

from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect.csr            import *

from gateware.lms7002.lms7002_rxiq import LMS7002RXIQ
from gateware.lms7002.lms7002_txiq import LMS7002TXIQ
from gateware.lms7002.lms7002_clk  import LMS7002CLK

# LMS7002 Top --------------------------------------------------------------------------------------

class LMS7002Top(LiteXModule):
    def __init__(self, platform, pads=None, hw_ver=None, add_csr=True, fpgacfg_manager=None, diq_width=12):

        assert pads            is not None
        assert hw_ver          is not None
        assert fpgacfg_manager is not None

        self.axis_m            = AXIStreamInterface(4 * diq_width, 8, clock_domain="lms_rx")
        self.axis_s            = AXIStreamInterface(128,              clock_domain="lms_tx")

        self.platform          = platform

        self.pads                = pads
        self.periph_output_val_1 = Signal(16)

        self.pct_sync_pulse      = Signal() # From RXTX
        self.pct_buff_rdy        = Signal() # From RXTX

        self.from_tstcfg_test_en      = Signal(6)
        self.from_tstcfg_tx_tst_i     = Signal(16)
        self.from_tstcfg_tx_tst_q     = Signal(16)

        self.tx_txant_en      = Signal()

        self.delay_ctrl_en    = Signal()
        self.delay_ctrl_sel   = Signal(2)
        self.delay_ctrl_dir   = Signal()
        self.delay_ctrl_mode  = Signal()
        self.delay_ctrl_done  = Signal()
        self.delay_ctrl_error = Signal()
        self.hw_ver           = Signal(4)

        self.smpl_cnt_en      = Signal() # To rx_path
        self.smpl_cmp_length  = Signal(16)
        self.smpl_cmp_cnt     = Signal(16) # Unused

        self.smpl_width       = Signal()

        # # #

        # Clock Domains.
        # --------------
        self.cd_lms_rx = ClockDomain()
        self.cd_lms_tx = ClockDomain()

        # Signals.
        # --------
        delay_sel           = Signal(2)

        inst0_loadn         = Signal()
        inst0_move          = Signal()

        inst1_delayf_loadn  = Signal()
        inst1_delayf_move   = Signal()

        tx_data_loadn       = Signal()
        tx_data_move        = Signal()

        rx_reset_n          = Signal()

        rx_data_loadn       = Signal()
        rx_data_move        = Signal()

        rx_ptrn_en          = Signal()

        rx_mode             = Signal()
        rx_trxiqpulse       = Signal()
        rx_ddr_en           = Signal()
        rx_mimo_en          = Signal()
        rx_ch_en            = Signal(2)

        tx_reset_n          = Signal()

        tx_ptrn_en          = Signal()

        tx_mode             = Signal()
        tx_trxiqpulse       = Signal()
        tx_ddr_en           = Signal()
        tx_mimo_en          = Signal()
        tx_ch_en            = Signal(2)
        tx_diq_h            = Signal(diq_width + 1)
        tx_diq_l            = Signal(diq_width + 1)

        smpl_cmp_en         = Signal()
        smpl_cmp_done       = Signal()
        smpl_cmp_error      = Signal()
        smpl_cmp_length     = Signal(16)


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
        # ------------------------------------------------------------------------------------------
        self.lms7002_txiq = ClockDomainsRenamer("lms_tx")(LMS7002TXIQ(12, pads))

        self.specials += [
            # FIFO 2 DIQ
            Instance("fifo2diq",
                # Parameters.
                p_iq_width            = diq_width,

                # Clk/Reset.
                i_clk                 = ClockSignal("lms_tx"),
                i_reset_n             = tx_reset_n,

                # Mode settings.
                i_mode                = tx_mode,
                i_trxiqpulse          = tx_trxiqpulse,
                i_ddr_en              = tx_ddr_en,
                i_mimo_en             = tx_mimo_en,
                i_ch_en               = tx_ch_en,
                i_fidm                = Constant(0, 1),
                i_pct_sync_mode       = fpgacfg_manager.synch_mode,
                i_pct_sync_pulse      = self.pct_sync_pulse,
                i_pct_sync_size       = fpgacfg_manager.sync_size,
                i_pct_buff_rdy        = self.pct_buff_rdy,

                # txant
                i_txant_cyc_before_en = fpgacfg_manager.txant_pre,
                i_txant_cyc_after_en  = fpgacfg_manager.txant_post,
                o_txant_en            = self.tx_txant_en,
                o_DIQ                 = Open(diq_width),
                o_fsync               = Open(),
                o_DIQ_h               = tx_diq_h,
                o_DIQ_l               = tx_diq_l,

                # AXI Stream Slave Interface.
                i_axis_s_tdata        = self.axis_s.data,
                i_axis_s_tvalid       = self.axis_s.valid,
                o_axis_s_tready       = self.axis_s.ready,
                i_axis_s_tlast        = self.axis_s.last,
            ),
            # txiqmux instance.
            # -----------------
            Instance("txiqmux",
                p_diq_width   = diq_width,
                i_clk               = ClockSignal("lms_tx"),
                i_reset_n           = ~ResetSignal("sys"),
                i_test_ptrn_en      = tx_ptrn_en,                # Enables test pattern
                i_test_ptrn_fidm    = Constant(0, 1),            # External Frame ID mode. Frame start at fsync = 0, when 0. Frame start at fsync = 1, when 1.
                i_test_ptrn_I       = self.from_tstcfg_tx_tst_i,
                i_test_ptrn_Q       = self.from_tstcfg_tx_tst_q,
                i_test_data_en      = fpgacfg_manager.tx_cnt_en,
                i_test_data_mimo_en = Constant(1, 1),
                i_mux_sel           = fpgacfg_manager.wfm_play,  # Mux select: 0 - tx, 1 - wfm
                i_tx_diq_h          = tx_diq_h,
                i_tx_diq_l          = tx_diq_l,
                i_wfm_diq_h         = Constant(0, diq_width + 1),
                i_wfm_diq_l         = Constant(0, diq_width + 1),
                o_diq_h             = self.lms7002_txiq.tx_diq1_h,
                o_diq_l             = self.lms7002_txiq.tx_diq1_l,
            ),
        ]




        self.comb += [
            # Delay control
            self.lms7002_txiq.data_loadn.eq(    tx_data_loadn),
            self.lms7002_txiq.data_move.eq(     tx_data_move),
            self.lms7002_txiq.data_direction.eq(0),
        ]

        self.specials += [
            MultiReg(fpgacfg_manager.rx_en,       tx_reset_n,      odomain="lms_tx"),
            MultiReg(fpgacfg_manager.rx_ptrn_en,  tx_ptrn_en,      odomain="lms_tx"),
            MultiReg(fpgacfg_manager.mode,        tx_mode,         odomain="lms_tx"),
            MultiReg(fpgacfg_manager.trxiq_pulse, tx_trxiqpulse,   odomain="lms_tx"),
            MultiReg(fpgacfg_manager.ddr_en,      tx_ddr_en,       odomain="lms_tx"),
            MultiReg(fpgacfg_manager.mimo_int_en, tx_mimo_en,      odomain="lms_tx"),
            MultiReg(fpgacfg_manager.ch_en,       tx_ch_en,        odomain="lms_tx"),
            MultiReg(self.smpl_cmp_length,        smpl_cmp_length, odomain="lms_tx"),
        ]

        # RX path (DIQ2).
        # ------------------------------------------------------------------------------------------

        self.lms7002_rxiq = ClockDomainsRenamer("lms_rx")(LMS7002RXIQ(12, pads))
        self.comb += [
            self.lms7002_rxiq.data_loadn.eq(    rx_data_loadn),
            self.lms7002_rxiq.data_move.eq(     rx_data_move),
            self.lms7002_rxiq.data_direction.eq(Constant(0, 1)),
        ]

        # lms7002_rx.
        # -----------
        self.specials += [
            Instance("diq2fifo",
                # Parameters.
                p_iq_width       = diq_width,
                p_invert_input_clocks = "OFF",
                # Clk/Reset.
                i_clk            = ClockSignal("lms_rx"),
                i_reset_n        = rx_reset_n,
                # Mode settings
                i_test_ptrn_en   = rx_ptrn_en,
                i_mode           = rx_mode,        # JESD207: 1; TRXIQ: 0
                i_trxiqpulse     = rx_trxiqpulse,  # trxiqpulse on: 1; trxiqpulse off: 0
                i_ddr_en         = rx_ddr_en,      # DDR: 1; SDR: 0
                i_mimo_en        = rx_mimo_en,     # SISO: 1; MIMO: 0
                i_ch_en          = rx_ch_en,       # "01" - Ch. A, "10" - Ch. B, "11" - Ch. A and Ch. B.
                i_fidm           = Constant(0, 1), # External Frame ID mode. Frame start at fsync = 0, when 0. Frame start at fsync = 1, when 1.
                # Rx interface data
                i_rx_diq2_h      = self.lms7002_rxiq.rx_diq2_h,
                i_rx_diq2_l      = self.lms7002_rxiq.rx_diq2_l,

                # AXI Stream Master Interface.
                o_m_axis_tdata   = self.axis_m.data,
                o_m_axis_tkeep   = self.axis_m.keep,
                o_m_axis_tvalid  = self.axis_m.valid,
                o_m_axis_tlast   = self.axis_m.last,
                i_m_axis_tready  = self.axis_m.ready,

                # sample compare
                i_smpl_cmp_start  = smpl_cmp_en,
                i_smpl_cmp_length = smpl_cmp_length,
                o_smpl_cmp_done   = smpl_cmp_done,
                o_smpl_cmp_err    = smpl_cmp_error,
                ## sample counter enable
                o_smpl_cnt_en     = self.smpl_cnt_en,
            )
        ]
        self.specials += [
            MultiReg(fpgacfg_manager.rx_en,       rx_reset_n,      odomain="lms_rx"),
            MultiReg(fpgacfg_manager.rx_ptrn_en,  rx_ptrn_en,      odomain="lms_rx"),
            MultiReg(fpgacfg_manager.mode,        rx_mode,         odomain="lms_rx"),
            MultiReg(fpgacfg_manager.trxiq_pulse, rx_trxiqpulse,   odomain="lms_rx"),
            MultiReg(fpgacfg_manager.ddr_en,      rx_ddr_en,       odomain="lms_rx"),
            MultiReg(fpgacfg_manager.mimo_int_en, rx_mimo_en,      odomain="lms_rx"),
            MultiReg(fpgacfg_manager.ch_en,       rx_ch_en,        odomain="lms_rx"),
            MultiReg(self.smpl_cmp_length,        smpl_cmp_length, odomain="lms_rx"),
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
            o_smpl_cmp_en      = smpl_cmp_en,
            i_smpl_cmp_done    = smpl_cmp_done,
            i_smpl_cmp_error   = smpl_cmp_error,
            o_smpl_cmp_cnt     = self.smpl_cmp_cnt,

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

        self.add_source(platform)

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

    def add_source(self, platform):
        lms7002_files = [
            # RX
            "gateware/hdl/rx_path_top/diq2fifo/synth/diq2fifo.vhd",
            "gateware/hdl/rx_path_top/smpl_cmp/synth/smpl_cmp.vhd",
            "gateware/hdl/rx_path_top/diq2fifo/synth/test_data_dd.vhd",
            "gateware/LimeDFB/lms7002/src/lms7002_rx.vhd",

            # TX
            "gateware/LimeDFB/lms7002/src/lms7002_tx.vhd",
            "gateware/LimeDFB/tx_path_top/src/sample_unpack.vhd",
            "gateware/hdl/tx_path_top/fifo2diq/synth/fifo2diq.vhd",
            "gateware/hdl/tx_path_top/fifo2diq/synth/txiq_ctrl.vhd",
        ]

        for file in lms7002_files:
            platform.add_source(file)

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

