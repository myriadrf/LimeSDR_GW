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

from litex.soc.interconnect                import stream
from litex.soc.interconnect.axi.axi_stream import AXIStreamInterface
from litex.soc.interconnect.csr            import *

from gateware.common                                  import add_vhd2v_converter

from gateware.LimeDFB_LiteX.lms7002.src.lms7002_ddin  import LMS7002DDIN
from gateware.LimeDFB_LiteX.lms7002.src.lms7002_ddout import LMS7002DDOUT
from gateware.LimeDFB_LiteX.lms7002.src.lms7002_clk   import LMS7002CLK

# LMS7002 Top --------------------------------------------------------------------------------------

class LMS7002Top(LiteXModule):
    def __init__(self, platform, pads=None, hw_ver=None, add_csr=True,
        fpgacfg_manager      = None,
        pllcfg_manager       = None,
        diq_width            = 12,
        invert_input_clock   = False,
        s_clk_domain         = "lms_tx",
        s_axis_tx_fifo_words = 16,
        m_clk_domain         = "lms_rx",
        m_axis_rx_fifo_words = 16,
        with_max10_pll       = False,
        ):

        assert pads            is not None
        assert hw_ver          is not None
        assert fpgacfg_manager is not None

        self.source            = AXIStreamInterface(64, clock_domain=m_clk_domain)
        self.sink              = AXIStreamInterface(64, clock_domain=s_clk_domain)

        self.platform          = platform

        self.pads                = pads
        self.periph_output_val_1 = Signal(16)

        self.from_tstcfg_test_en      = Signal(6)
        self.from_tstcfg_tx_tst_i     = Signal(16)
        self.from_tstcfg_tx_tst_q     = Signal(16)

        self.delay_ctrl_sel   = Signal(2)
        self.delay_ctrl_done  = Signal()
        self.delay_ctrl_error = Signal()
        self.hw_ver           = Signal(4)

        self.smpl_cnt_en      = Signal() # To rx_path
        self.smpl_cmp_cnt     = Signal(16) # Unused

        # CSR --------------------------------------------------------------------------------------
        # LMS Ctrl GPIO
        self._lms_ctr_gpio = CSRStorage(size=4, description="LMS Control GPIOs.")

        # fpgacfg
        self.lms1              = CSRStorage(fields=[         # 19
            CSRField("ss",             size=1, offset=0, reset=1),
            CSRField("reset",          size=1, offset=1, reset=1),
            CSRField("core_ldo_en",    size=1, offset=2, reset=0),
            CSRField("txnrx1",         size=1, offset=3, reset=1),
            CSRField("txnrx2",         size=1, offset=4, reset=0),
            CSRField("txen",           size=1, offset=5, reset=1),
            CSRField("rxen",           size=1, offset=6, reset=1),
            # FIXME: not sure
            CSRField("txrxen_mux_sel", size=1, offset=7, reset=0),
            CSRField("txrxen_inv",     size=1, offset=8, reset=0),

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

        if platform.name not in ["limesdr_mini_v1", "limesdr_mini_v2"]:
            self.cmp_start = CSRStorage(1, reset=0,
                description="Start sample compare: 0: idle, 1 transition: start configuration"
            )
            self.cmp_length = CSRStorage(16, reset=0xEFFF,
                description="Sample compare length"
            )
            self.cmp_done = CSRStatus(1,
                description="Sample compare done: 0: Not done, 1: Done"
            )
            self.cmp_error = CSRStatus(1,
                description="Sample compare error: 0: No error, 1: Error"
            )

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

        rx_reset_n          = Signal()

        rx_test_data_h      = Signal(diq_width + 1)
        rx_test_data_l      = Signal(diq_width + 1)
        rx_diq2_h_mux       = Signal(diq_width + 1)
        rx_diq2_l_mux       = Signal(diq_width + 1)

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
        tx_test_data_h      = Signal(diq_width + 1)
        tx_test_data_l      = Signal(diq_width + 1)

        tx_tst_ptrn_h       = Signal(diq_width + 1)
        tx_tst_ptrn_l       = Signal(diq_width + 1)
        tx_mux_sel          = Signal()
        tx_tst_data_en      = Signal()

        smpl_cmp_en         = Signal()
        smpl_cmp_done       = Signal()
        smpl_cmp_error      = Signal()
        smpl_cmp_cnt        = Signal(16)
        smpl_cmp_length     = Signal(16)
        rx_smpl_cmp_length  = Signal(16)
        tx_smpl_cmp_length  = Signal(16)

        # LiteScope Probes.
        self.smpl_cmp_en    = Signal()
        self.smpl_cmp_done  = Signal()
        self.smpl_cmp_error = Signal()
        self.smpl_cmp_length= Signal(16)

        # Sync lms_rx <-> sys.
        smpl_cmp_en_sync    = Signal()
        smpl_cmp_done_sync  = Signal()
        smpl_cmp_error_sync = Signal()
        smpl_cmp_length_sync = Signal(16)

        # Clocks.
        # -------
        self.lms7002_clk = LMS7002CLK(platform, pads, pllcfg_manager, with_max10_pll=with_max10_pll)
        self.comb += [
            self.cd_lms_tx.clk.eq(self.lms7002_clk.tx_clk),
            self.cd_lms_rx.clk.eq(self.lms7002_clk.rx_clk),
            # Configuration
            self.lms7002_clk.sel.eq(      Constant(0, 1)), # 0 - fclk1 control, 1 - fclk2 control
            self.lms7002_clk.direction.eq(Constant(0, 1)),
            self.lms7002_clk.loadn.eq(    inst0_loadn),
            self.lms7002_clk.move.eq(     inst0_move),
            # mini v1
            self.lms7002_clk.clk_ena.eq(    fpgacfg_manager.clk_ena),
            self.lms7002_clk.drct_clk_en.eq(Replicate(fpgacfg_manager.drct_clk_en[0], 4)),
        ]

        # TX Path (DIQ1).
        # ------------------------------------------------------------------------------------------
        self.lms7002_ddout = ClockDomainsRenamer("lms_tx")(LMS7002DDOUT(platform, 12, pads))
        self.tx_cdc        = stream.ClockDomainCrossing([("data", 64)], cd_from=s_clk_domain, cd_to="lms_tx", depth=s_axis_tx_fifo_words)

        self.lms7002_tx = Instance("LMS7002_TX",
            # Parameters.
            p_G_IQ_WIDTH      = diq_width,

            # Clk/Reset.
            i_CLK             = ClockSignal("lms_tx"),
            i_RESET_N         = tx_reset_n,

            # Mode settings
            i_MODE            = tx_mode,
            i_TRXIQPULSE      = tx_trxiqpulse,
            i_DDR_EN          = tx_ddr_en,
            i_MIMO_EN         = tx_mimo_en,
            i_CH_EN           = tx_ch_en,
            i_FIDM            = Constant(0, 1),

            # Tx interface data.
            o_DIQ_H             = tx_diq_h,
            o_DIQ_L             = tx_diq_l,

            # AXI Stream Slave Interface.
            i_S_AXIS_ARESET_N = Constant(0, 1), # Unused
            i_S_AXIS_ACLK     = Constant(0, 1), # Unused
            i_S_AXIS_TVALID   = self.tx_cdc.source.valid,
            i_S_AXIS_TDATA    = self.tx_cdc.source.data,
            o_S_AXIS_TREADY   = self.tx_cdc.source.ready,
            i_S_AXIS_TLAST    = self.tx_cdc.source.last,
        )

        # test_data_dd.
        # -------------
        self.tx_test_data_dd = Instance("test_data_dd",
            # Clk/Reset.
            i_clk       = ClockSignal("lms_tx"),
            i_reset_n   = ~ResetSignal("lms_tx"),
            # Mode Settings.
            i_fr_start  = Constant(0, 1), # External Frame ID mode. Frame start at fsync = 0, when 0. Frame start at fsync = 1, when 1.
            i_mimo_en   = tx_mimo_en,     # SISO: 1; MIMO: 0

            # Output.
            o_data_h    = tx_test_data_h,
            o_data_l    = tx_test_data_l,
        )

        # txiq_tst_ptrn.
        # --------------
        self.txiq_tst_ptrn = Instance("txiq_tst_ptrn",
            # Parameters.
            p_diq_width = diq_width,

            # Clk/Reset.
            i_clk       = ClockSignal("lms_tx"),
            i_reset_n   = tx_ptrn_en,

            # Output.
            o_diq_h     = tx_tst_ptrn_h,
            o_diq_l     = tx_tst_ptrn_l,
        )

        if platform.name.startswith("limesdr_mini"):
            self.txiq_tst_ptrn.update(
                # Mode Settings.
                i_fidm   = Constant(0, 1),            # External Frame ID mode. Frame start at fsync = 0, when 0. Frame start at fsync = 1, when 1.
                i_ptrn_i = self.from_tstcfg_tx_tst_i,
                i_ptrn_q = self.from_tstcfg_tx_tst_q,
            )

        self.specials += [
            MultiReg(fpgacfg_manager.tx_en,           tx_reset_n,      odomain="lms_tx"),
            MultiReg(fpgacfg_manager.tx_ptrn_en,      tx_ptrn_en,      odomain="lms_tx"),
            MultiReg(fpgacfg_manager.tx_cnt_en,       tx_tst_data_en,  odomain="lms_tx"),
            MultiReg(fpgacfg_manager.wfm_play,        tx_mux_sel,      odomain="lms_tx"),
            MultiReg(fpgacfg_manager.mode,            tx_mode,         odomain="lms_tx"),
            MultiReg(fpgacfg_manager.trxiq_pulse,     tx_trxiqpulse,   odomain="lms_tx"),
            MultiReg(fpgacfg_manager.ddr_en,          tx_ddr_en,       odomain="lms_tx"),
            MultiReg(fpgacfg_manager.mimo_int_en,     tx_mimo_en,      odomain="lms_tx"),
            MultiReg(fpgacfg_manager.ch_en,           tx_ch_en,        odomain="lms_tx"),
        ]
        if pllcfg_manager is not None:
            self.specials += MultiReg(pllcfg_manager.auto_phcfg_smpls, tx_smpl_cmp_length, odomain="lms_tx"),
        else:
            if platform.name.startswith("limesdr_mini"):
                self.comb += tx_smpl_cmp_length.eq(0)
            else:
                self.comb += tx_smpl_cmp_length.eq(smpl_cmp_cnt)

        # RX path (DIQ2). --------------------------------------------------------------------------
        self.lms7002_ddin = ClockDomainsRenamer("lms_rx")(LMS7002DDIN(platform, 12, pads, invert_input_clock))

        self.rx_cdc = stream.ClockDomainCrossing([("data", 64), ("keep", 64 // 8)],
            cd_from = "lms_rx",
            cd_to   = m_clk_domain,
            depth   = m_axis_rx_fifo_words,
        )

        # lms7002_rx.
        # -----------
        smpl_cmp_params = dict()
        smpl_cmp_params.update(
            # Parameters.
            p_smpl_width    = diq_width,

            # Clk/Reset.
            i_clk           = ClockSignal("lms_rx"),
            i_reset_n       = smpl_cmp_en,

            # DIQ bus.
            i_diq_h         = self.lms7002_ddin.rx_diq2_h,
            i_diq_l         = self.lms7002_ddin.rx_diq2_l,

            # Control signals
            i_cmp_start     = smpl_cmp_en,
            i_cmp_length    = rx_smpl_cmp_length,
            o_cmp_done      = smpl_cmp_done,
            o_cmp_error     = smpl_cmp_error,
            i_cmp_AI        = Constant(0xAAA, diq_width),
            i_cmp_AQ        = Constant(0x555, diq_width),
            i_cmp_BI        = Constant(0xAAA, diq_width),
            i_cmp_BQ        = Constant(0x555, diq_width),
        )

        if platform.name.startswith("limesdr_mini"):
            smpl_cmp_params.update(
                # Mode settings
                i_mode          = rx_mode,
                i_trxiqpulse    = rx_trxiqpulse,
                i_ddr_en        = rx_ddr_en,
                i_mimo_en       = rx_mimo_en,
                i_ch_en         = rx_ch_en,
                i_fidm          = Constant(0, 1),

                # Control signals
                o_cmp_error_cnt = Open(16),
            )
        else:
            self.DEBUG_IQ_ERR = Signal()
            self.DEBUG_AI_ERR = Signal()
            self.DEBUG_AQ_ERR = Signal()
            self.DEBUG_BI_ERR = Signal()
            self.DEBUG_BQ_ERR = Signal()

            smpl_cmp_params.update(
                o_DEBUG_IQ_ERR = self.DEBUG_IQ_ERR,
                o_DEBUG_AI_ERR = self.DEBUG_AI_ERR,
                o_DEBUG_AQ_ERR = self.DEBUG_AQ_ERR,
                o_DEBUG_BI_ERR = self.DEBUG_BI_ERR,
                o_DEBUG_BQ_ERR = self.DEBUG_BQ_ERR,
            )

        self.smpl_cmp = Instance("smpl_cmp", **smpl_cmp_params)

        # test_data_dd.
        # -------------
        self.rx_test_data_dd = Instance("test_data_dd",
            # Clk/Reset.
            i_clk       = ClockSignal("lms_rx"),
            i_reset_n   = rx_reset_n,
            # Mode Settings.
            i_fr_start  = Constant(0, 1), # External Frame ID mode. Frame start at fsync = 0, when 0. Frame start at fsync = 1, when 1.
            i_mimo_en   = rx_mimo_en,     # SISO: 1; MIMO: 0

            # Output.
            o_data_h    = rx_test_data_h,
            o_data_l    = rx_test_data_l,
        )

        rx_cdc_sink_valid = Signal()

        self.lms7002_rx = Instance("lms7002_rx",
            # Parameters.
            p_g_IQ_WIDTH          = diq_width,
            p_g_M_AXIS_FIFO_WORDS = 16,

            # Clock/Reset.
            i_clk                 = ClockSignal("lms_rx"),
            i_reset_n             = rx_reset_n,

            # Mode settings
            i_mode                = {True: rx_mode, False: Constant(0, 1)}[platform.name.startswith("limesdr_mini")],
            i_trxiqpulse          = rx_trxiqpulse,
            i_ddr_en              = rx_ddr_en,
            i_mimo_en             = rx_mimo_en,
            i_ch_en               = rx_ch_en,
            i_fidm                = Constant(0, 1),
            # Tx interface data
            i_diq_h               = rx_diq2_h_mux,
            i_diq_l               = rx_diq2_l_mux,

            # AXI Stream Master Interface.
            i_m_axis_areset_n     = Constant(1, 1),
            i_m_axis_aclk         = ClockSignal("lms_rx"),
            o_m_axis_tvalid       = rx_cdc_sink_valid,
            o_m_axis_tdata        = self.rx_cdc.sink.data,
            o_m_axis_tkeep        = self.rx_cdc.sink.keep,
            i_m_axis_tready       = self.rx_cdc.sink.ready,
            o_m_axis_tlast        = self.rx_cdc.sink.last
        )

        if platform.name.startswith("limesdr_mini"):
            self.specials += MultiReg(fpgacfg_manager.rx_en, rx_reset_n, odomain="lms_rx"),
        else:
            self.specials += MultiReg(fpgacfg_manager.tx_en, rx_reset_n, odomain="lms_rx"),

        self.specials += [
            MultiReg(fpgacfg_manager.rx_ptrn_en,      rx_ptrn_en,      odomain="lms_rx"),
            MultiReg(fpgacfg_manager.mode,            rx_mode,         odomain="lms_rx"),
            MultiReg(fpgacfg_manager.trxiq_pulse,     rx_trxiqpulse,   odomain="lms_rx"),
            MultiReg(fpgacfg_manager.ddr_en,          rx_ddr_en,       odomain="lms_rx"),
            MultiReg(fpgacfg_manager.mimo_int_en,     rx_mimo_en,      odomain="lms_rx"),
            MultiReg(fpgacfg_manager.ch_en,           rx_ch_en,        odomain="lms_rx"),
        ]
        if pllcfg_manager is not None:
            self.specials += MultiReg(pllcfg_manager.auto_phcfg_smpls, rx_smpl_cmp_length, odomain="lms_rx"),
        else:
            if platform.name.startswith("limesdr_mini"):
                self.comb += rx_smpl_cmp_length.eq(0)
            else:
                self.comb += rx_smpl_cmp_length.eq(smpl_cmp_cnt)

        # Delay control module.
        # ---------------------
        if platform.name in ["limesdr_mini_v2"]:
            self.delay_ctrl_top = Instance("delay_ctrl_top",
                # Clk/Reset.
                i_clk              = ClockSignal("lms_rx"),
                i_reset_n          = ~ResetSignal("lms_rx"),

                #
                i_delay_en         = self.reg03.fields.phcfg_start,
                i_delay_sel        = self.delay_ctrl_sel,
                i_delay_dir        = self.reg03.fields.phcfg_updn,
                i_delay_mode       = self.reg03.fields.phcfg_mode,
                o_delay_done       = self.delay_ctrl_done,
                o_delay_error      = self.delay_ctrl_error,

                # signals from sample compare module (required for automatic phase searching)
                o_smpl_cmp_en      = smpl_cmp_en,
                i_smpl_cmp_done    = smpl_cmp_done,
                i_smpl_cmp_error   = smpl_cmp_error,
                o_smpl_cmp_cnt     = smpl_cmp_cnt,

                o_delayf_loadn     = inst1_delayf_loadn,
                o_delayf_move      = inst1_delayf_move,
                o_delayf_direction = Open(),
            )
        elif platform.name in ["limesdr_mini_v1"]:
            self.comb += [
                smpl_cmp_en.eq(                    self.lms7002_clk.smpl_cmp_en),
                self.lms7002_clk.smpl_cmp_done.eq( smpl_cmp_done),
                self.lms7002_clk.smpl_cmp_error.eq(smpl_cmp_error),
                smpl_cmp_cnt.eq(                   self.lms7002_clk.smpl_cmp_cnt),
            ]
        else:
            self.specials += [
                MultiReg(self.cmp_start.storage,  smpl_cmp_en_sync,     odomain="lms_rx"),
                MultiReg(self.cmp_length.storage, smpl_cmp_length_sync, odomain="lms_rx"),
                MultiReg(smpl_cmp_done,           smpl_cmp_done_sync,   odomain="sys"),
                MultiReg(smpl_cmp_error,          smpl_cmp_error_sync,  odomain="sys"),
            ]
            self.comb += [
                smpl_cmp_en.eq(          smpl_cmp_en_sync),
                smpl_cmp_cnt.eq(         smpl_cmp_length_sync),
                self.cmp_done.status.eq( smpl_cmp_done_sync),
                self.cmp_error.status.eq(smpl_cmp_error_sync),

                # LiteScope Probes.
                self.smpl_cmp_en.eq(     smpl_cmp_en_sync),
                self.smpl_cmp_cnt.eq(    smpl_cmp_length_sync),
                self.smpl_cmp_done.eq(   smpl_cmp_done),
                self.smpl_cmp_error.eq(  smpl_cmp_error),
            ]

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
            self.sink.connect(self.tx_cdc.sink, keep=["data", "ready", "last", "valid"]),
            self.lms7002_ddout.data_direction.eq(0),
            If(self.delay_ctrl_sel == 0b01,
                self.lms7002_ddout.data_loadn.eq(inst1_delayf_loadn),
                self.lms7002_ddout.data_move.eq (inst1_delayf_move),
            ).Else(
                self.lms7002_ddout.data_loadn.eq(1),
                self.lms7002_ddout.data_move.eq (0),
            ),
            # lms7002_rx
            self.rx_cdc.source.connect(self.source),
            self.rx_cdc.sink.valid.eq(rx_cdc_sink_valid),
            If(~rx_reset_n,
               self.rx_cdc.source.ready.eq(1),
               self.source.valid.eq(0),
               self.rx_cdc.sink.valid.eq(0),
            ),
            self.lms7002_ddin.data_direction.eq(Constant(0, 1)),
            If(self.delay_ctrl_sel == 0b11,
                self.lms7002_ddin.data_loadn.eq(inst1_delayf_loadn),
                self.lms7002_ddin.data_move.eq (inst1_delayf_move),
            ).Else(
                self.lms7002_ddin.data_loadn.eq(1),
                self.lms7002_ddin.data_move.eq (0),
            ),
            # Pads.
            self.pads.CORE_LDO_EN.eq(self.lms1.fields.core_ldo_en),
            self.pads.TXNRX1.eq(     self.lms1.fields.txnrx1),

            # pllcfg
            self.reg01.status.eq(Cat(1, 0, self.delay_ctrl_done, self.delay_ctrl_error, Constant(0, 12))),
            If(self.reg03.fields.cnt_ind == 0b0011,
                self.delay_ctrl_sel.eq(0),
            ).Else(
                self.delay_ctrl_sel.eq(3),
            ),
        ]

        # RX sync
        self.sync.lms_rx += [
            If(rx_ptrn_en,
               rx_diq2_h_mux.eq(rx_test_data_h),
               rx_diq2_l_mux.eq(rx_test_data_l),
            ).Else(
               rx_diq2_h_mux.eq(self.lms7002_ddin.rx_diq2_h),
               rx_diq2_l_mux.eq(self.lms7002_ddin.rx_diq2_l),
            ),
            # Sample counter.
            If(~rx_mimo_en & rx_ddr_en,
                self.smpl_cnt_en.eq(1)
            ).Else(
                self.smpl_cnt_en.eq(~self.smpl_cnt_en)
            ),
            If(~smpl_cmp_en & ~rx_reset_n,
                self.smpl_cnt_en.eq(0),
            )
        ]

        if platform.name.startswith("limesdr_mini"):
            # TX sync
            self.sync.lms_tx += [
                If(tx_tst_data_en,
                    self.lms7002_ddout.tx_diq1_h.eq(tx_test_data_h),
                    self.lms7002_ddout.tx_diq1_l.eq(tx_test_data_l),
                ).Elif(tx_ptrn_en,
                    self.lms7002_ddout.tx_diq1_h.eq(tx_tst_ptrn_h),
                    self.lms7002_ddout.tx_diq1_l.eq(tx_tst_ptrn_l),
                ).Elif(tx_mux_sel,
                    self.lms7002_ddout.tx_diq1_h.eq(Constant(0, diq_width + 1)),
                    self.lms7002_ddout.tx_diq1_l.eq(Constant(0, diq_width + 1)),
                ).Else(
                    self.lms7002_ddout.tx_diq1_h.eq(tx_diq_h),
                    self.lms7002_ddout.tx_diq1_l.eq(tx_diq_l),
                ),
            ]
        else:
            self.comb += [
                If(tx_ptrn_en,
                    self.lms7002_ddout.tx_diq1_h.eq(tx_tst_ptrn_h),
                    self.lms7002_ddout.tx_diq1_l.eq(tx_tst_ptrn_l),
                ).Else(
                    self.lms7002_ddout.tx_diq1_h.eq(tx_diq_h),
                    self.lms7002_ddout.tx_diq1_l.eq(tx_diq_l),
                )
            ]


        # LMS Controls.
        if platform.name.startswith("limesdr_mini"):
            self.comb += [
                self.pads.TXEN.eq( self.lms1.fields.txen),
                self.pads.RXEN.eq( self.lms1.fields.rxen),
                self.pads.RESET.eq(self.lms1.fields.reset & self._lms_ctr_gpio.storage[0]),
            ]
            if platform.name in ["limesdr_mini_v2"]:
                self.comb += [
                    If((self.hw_ver > 5),
                        self.pads.TXNRX2_or_CLK_SEL.eq(self.periph_output_val_1),
                    ).Else(
                        self.pads.TXNRX2_or_CLK_SEL.eq(self.lms1.fields.txnrx2),
                    ),
                ]
            else:
                self.comb += self.pads.TXNRX2.eq(self.lms1.fields.txnrx2)
        else:
            lms_txen = Signal()
            lms_rxen = Signal()
            txant_en = Signal()
            self.comb += [
                self.pads.TXNRX2_or_CLK_SEL.eq(self.lms1.fields.txnrx2),
                self.pads.RESET.eq(            self.lms1.fields.reset),
                If(self.lms1.fields.txrxen_mux_sel,
                    lms_txen.eq(txant_en),
                    lms_rxen.eq(~txant_en),
                ).Else(
                    lms_txen.eq(self.lms1.fields.txen),
                    lms_rxen.eq(self.lms1.fields.rxen),
                ),
                If(self.lms1.fields.txrxen_inv,
                   self.pads.TXEN.eq(~lms_txen),
                   self.pads.RXEN.eq(~lms_rxen),
                ).Else(
                   self.pads.TXEN.eq(lms_txen),
                   self.pads.RXEN.eq(lms_rxen),
                )
            ]

        self.specials += AsyncResetSynchronizer(self.cd_lms_rx, ResetSignal("sys"))
        self.specials += AsyncResetSynchronizer(self.cd_lms_tx, ResetSignal("sys"))

        # TX.
        # ---

        # LMS7002_TX
        self.lms7002_tx_conv = add_vhd2v_converter(self.platform,
            instance = self.lms7002_tx,
            files    = ["gateware/LimeDFB/lms7002/src/lms7002_tx.vhd"],
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.lms7002_tx)

        # TX test_data_dd.
        self.tx_test_data_dd_conv = add_vhd2v_converter(self.platform,
            instance = self.tx_test_data_dd,
            files    = ["gateware/LimeDFB_LiteX/lms7002/src/test_data_dd.vhd"],
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.tx_test_data_dd)

        # txiq_tst_ptrn.
        if self.platform.name.startswith("limesdr_mini"):
            txiq_tst_ptrn_files = [
                "gateware/LimeDFB_LiteX/lms7002/src/txiq_tst_ptrn.vhd",
                "gateware/LimeDFB_LiteX/general/sync_reg.vhd",
                "gateware/LimeDFB_LiteX/general/bus_sync_reg.vhd",
            ]
        else:
            txiq_tst_ptrn_files = ["gateware/LimeDFB/lms7002/src/txiq_tst_ptrn.vhd"]

        self.txiq_tst_ptrn_conv = add_vhd2v_converter(self.platform,
            instance = self.txiq_tst_ptrn,
            files    = txiq_tst_ptrn_files,
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.txiq_tst_ptrn)

        # RX.
        # ---

        # Smpl CMP.
        if self.platform.name.startswith("limesdr_mini"):
            smpl_cmp_file = "gateware/LimeDFB_LiteX/lms7002/src/smpl_cmp.vhd"
        else:
            smpl_cmp_file = "gateware/LimeDFB_LiteX/lms7002/src/smpl_cmp_xtrx.vhd"
        self.smpl_cmp_conv = add_vhd2v_converter(self.platform,
            instance = self.smpl_cmp,
            files    = [smpl_cmp_file],
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.smpl_cmp)

        # RX test_data_dd.
        self.rx_test_data_dd_conv = add_vhd2v_converter(self.platform,
            instance = self.rx_test_data_dd,
            files    = ["gateware/LimeDFB_LiteX/lms7002/src/test_data_dd.vhd"],
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.rx_test_data_dd)

        # LMS7002RX.
        self.lms7002_rx_conv = add_vhd2v_converter(self.platform,
            instance = self.lms7002_rx,
            files    = ["gateware/LimeDFB/lms7002/src/lms7002_rx.vhd"],
        )
        # Removed Instance to avoid multiple definition
        self._fragment.specials.remove(self.lms7002_rx)

        # Delay Ctrl.
        if hasattr(self, "delay_ctrl_top"):
            delay_ctrl_top_files = [
                "gateware/LimeDFB_LiteX/delayf_ctrl/delay_ctrl_fsm.vhd",
                "gateware/LimeDFB_LiteX/delayf_ctrl/delay_ctrl_top.vhd",
                "gateware/LimeDFB_LiteX/delayf_ctrl/delayf_ctrl.vhd",
            ]

            self.delay_ctrl_top_conv = add_vhd2v_converter(self.platform,
                instance = self.delay_ctrl_top,
                files    = delay_ctrl_top_files,
            )
            # Removed Instance to avoid multiple definition
            self._fragment.specials.remove(self.delay_ctrl_top)
