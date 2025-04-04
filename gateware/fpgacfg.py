#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from gateware.common import FromFPGACfg

# FPGA Cfg -----------------------------------------------------------------------------------------

class FPGACfg(LiteXModule):
    def __init__(self, platform, board_id, major_rev, compile_rev, pads=None):
        self.from_fpgacfg      = FromFPGACfg()
        self.pwr_src           = Signal()
        self.soc_has_timesource = False,

        # Export.
        # -------
        self.ch_en             = Signal(2)
        self.smpl_width        = Signal(2)
        self.mode              = Signal()
        self.ddr_en            = Signal()
        self.trxiq_pulse       = Signal()
        self.mimo_int_en       = Signal()
        self.synch_dis         = Signal()
        self.synch_mode        = Signal()
        self.smpl_nr_clr       = Signal()
        self.txpct_loss_clr    = Signal()
        self.rx_en             = Signal()
        self.tx_en             = Signal()
        self.rx_ptrn_en        = Signal()
        self.tx_ptrn_en        = Signal()
        self.tx_cnt_en         = Signal()
        self.wfm_play          = Signal()
        self.sync_pulse_period = Signal(32)
        self.sync_size         = Signal(16)

        self.txant_pre         = Signal(16)
        self.txant_post        = Signal(16)

        self.tcxo_en           = Signal()
        self.ext_clk           = Signal()

        self.drct_clk_en       = Signal(16)
        self.clk_ena           = Signal(16)

        # Stream delay signals
        # --------------
        self.rx_en_delay_mode   = Signal(2)
        self.rx_en_delay_signal = Signal(2)
        rx_en_delay     = Signal(reset=0)
        self.tx_en_delay_mode   = Signal(2)
        self.tx_en_delay_signal = Signal(2)
        tx_en_delay     = Signal(reset=0)

        # CSRs.
        # -----

        # Read only registers (0-3)
        self.board_id          = CSRStatus(16,  reset=board_id)
        self.major_rev         = CSRStatus(16,  reset=major_rev)
        self.compile_rev       = CSRStatus(16,  reset=compile_rev)

        # limesdr_mini only
        if platform.name in ["limesdr_mini_v1", "limesdr_mini_v2"]:
            self.bom_hw_ver    = CSRStatus(16,  reset=0)

            # FPGA direct clocking (4-6)
            self.phase_reg_sel = CSRStorage(16, reset=0)
            self._drct_clk_en  = CSRStorage(16, reset=0)
            # load_phase, cnt_int[4:0], clk_ind[4:0]
            self.load_phase    = CSRStorage(fields=[
                CSRField("clk_int",        size=5, offset=0),
                CSRField("cnt_int",        size=5, offset=5),
                CSRField("load_phase_reg", size=1, offset=10),
            ])
        else:
            self.reserved_03 = CSRStorage(16, reset=0)
            self.reserved_04 = CSRStorage(16, reset=0)
            self.reserved_05 = CSRStorage(16, reset=0)
            self.reserved_06 = CSRStorage(16, reset=0)

        # Interface config (7-15)
        self._ch_en            = CSRStorage(2,  reset=0b11,
            description="2b01 - Channel A, 2b10 - Channel B enabled, 2b11 - Channels A and B"
        )
        self.reg08             = CSRStorage(fields=[
            CSRField("smpl_width",  size=2, offset=0,  reset=0b10),
            CSRField("mode",        size=1, offset=5,  reset=0),
            CSRField("ddr_en",      size=1, offset=6,  reset={True:0, False:1}[platform.name.startswith("limesdr_mini")]),
            CSRField("trxiq_pulse", size=1, offset=7,  reset=0),
            CSRField("mimo_int_en", size=1, offset=8,  reset=1),
            CSRField("synch_dis",   size=1, offset=9,  reset={True:0, False:1}[platform.name.startswith("limesdr_mini")]),
            CSRField("synch_mode",  size=1, offset=10, reset=0),
        ])
        self.reg09             = CSRStorage(fields=[
            CSRField("smpl_nr_clr",    size=1, offset=0, reset={True:1, False:0}[platform.name.startswith("limesdr_mini")]),
            CSRField("txpct_loss_clr", size=1, offset=1, reset=1),
        ])
        self.reg10             = CSRStorage(fields=[
            CSRField("rx_en",       size=1, offset=0),
            CSRField("tx_en",       size=1, offset=1),
            CSRField("rx_ptrn_en" , size=1, offset=8),
            CSRField("tx_ptrn_en",  size=1, offset=9),
            CSRField("tx_cnt_en",   size=1, offset=10),
        ])
        self.wfm_ch_en         = CSRStorage(16, reset=0b11)   # 12
        self.reg13             = CSRStorage(fields=[
            CSRField("wfm_play", size=1, offset=1),
            CSRField("wfm_load", size=1, offset=2),
        ])
        self.wfm_smpl_width    = CSRStorage(2,  reset=0b10)   # 14
        self._sync_size        = CSRStorage(16, reset=0x03FC) # 15

        # Peripheral config (16-31).
        self._txant_pre         = CSRStorage(16, reset=1)
        self._txant_post        = CSRStorage(16, reset=1)
        if platform.name.startswith("limesdr_mini"):
            self.spi_ss             = CSRStorage(16, reset=0xffff)
        else:
            self.reg18          = CSRStorage(fields=[
                CSRField("tcxo_en", size=1, offset=1, reset=1,
                    description="TCXO Enable: 0: Disabled, 1: Enabled."),
                CSRField("ext_clk", size=1, offset=2, reset=0,
                    description="CLK source select: 0: Internal, 1: External."),
            ])
        self._clk_ena           = CSRStorage(4, reset=0b1111)              # 29
        self._sync_pulse_period = CSRStorage(32, reset=0x3D090)            # 30

        # # #

        # Signals.
        _bom_ver = Signal(3)
        _hw_ver  = Signal(4)

        # Logic.
        if platform.name in ["limesdr_mini_v1", "limesdr_mini_v2"]:
            self.comb += [
                self.bom_hw_ver.status.eq(          Cat(_hw_ver, _bom_ver, self.pwr_src, Constant(0, 8))),
                # FPGA direct clocking
                self.from_fpgacfg.phase_reg_sel.eq( self.phase_reg_sel.storage),
                self.from_fpgacfg.drct_clk_en.eq(   self._drct_clk_en.storage),
                self.drct_clk_en.eq(                self._drct_clk_en.storage),
                self.from_fpgacfg.load_phase_reg.eq(self.load_phase.fields.load_phase_reg),
                self.from_fpgacfg.clk_ind.eq(       self.load_phase.fields.clk_int),
                self.from_fpgacfg.cnt_ind.eq(       self.load_phase.fields.cnt_int),
                # Peripheral config.
                self.from_fpgacfg.spi_ss.eq(        self.spi_ss.storage),
            ]
            self.sync += [
                If((pads.HW_VER == 0),
                    _bom_ver.eq(Cat(pads.BOM_VER[0:2], Constant(0,2))),
                    If(pads.BOM_VER[2] == 0b1,
                        _hw_ver.eq(Constant(2, 4)),
                    ).Else(
                        _hw_ver.eq(Constant(1, 4)),
                    ),
                ).Else(
                    _hw_ver.eq(pads.HW_VER),
                    _bom_ver.eq(pads.BOM_VER),
                ),
            ]
        else:
            self.comb += [
                self.tcxo_en.eq(self.reg18.fields.tcxo_en),
                self.ext_clk.eq(self.reg18.fields.ext_clk),
            ]

        self.comb += [

            self.from_fpgacfg.wfm_ch_en.eq(        self.wfm_ch_en.storage),
            self.from_fpgacfg.wfm_smpl_width.eq(   self.wfm_smpl_width.storage),

            # Peripheral config.
            self.from_fpgacfg.clk_ena.eq(          self._clk_ena.storage),
            self.clk_ena.eq(                       self._clk_ena.storage),

            # export.
            # -------
            # FPGA Cfg.
            self.ch_en.eq(            self._ch_en.storage),
            self.smpl_width.eq(       self.reg08.fields.smpl_width),
            self.mode.eq(             self.reg08.fields.mode),
            self.ddr_en.eq(           self.reg08.fields.ddr_en),
            self.trxiq_pulse.eq(      self.reg08.fields.trxiq_pulse),
            self.mimo_int_en.eq(      self.reg08.fields.mimo_int_en),
            self.synch_dis.eq(        self.reg08.fields.synch_dis),
            self.synch_mode.eq(       self.reg08.fields.synch_mode),
            self.smpl_nr_clr.eq(      self.reg09.fields.smpl_nr_clr),
            self.txpct_loss_clr.eq(   self.reg09.fields.txpct_loss_clr),
            self.tx_ptrn_en.eq(       self.reg10.fields.tx_ptrn_en),
            self.rx_ptrn_en.eq(       self.reg10.fields.rx_ptrn_en),
            self.tx_cnt_en.eq(        self.reg10.fields.tx_cnt_en),
            self.wfm_play.eq(         self.reg13.fields.wfm_play),
            self.sync_pulse_period.eq(self._sync_pulse_period.storage),
            self.sync_size.eq(        self._sync_size.storage),
            # Peripheral config.
            self.txant_pre.eq(        self._txant_pre.storage),
            self.txant_post.eq(       self._txant_post.storage),

        ]
        if self.soc_has_timesource:
            # Stream delay logic
            # Mode 0 uses no delay
            # Mode 1 uses delay signal 0
            # Mode 2 uses delay signal 1
            # ...
            # tx_en_delay and rx_en_delay are outputs of mux
            self.sync += [
                # Add option to delay stream start until a delay signal is pulsed high
                If(self.tx_en_delay_mode != 0,[
                    If(self.reg10.fields.tx_en == 0,[
                        self.tx_en.eq(0),
                    ]).Else([
                        self.tx_en.eq(self.tx_en | tx_en_delay),
                    ])
                ]).Else([
                    self.tx_en.eq(            self.reg10.fields.tx_en),
                ]),
                # Add option to delay stream start until a delay signal is pulsed high
                If(self.rx_en_delay_mode != 0, [
                    If(self.reg10.fields.rx_en == 0,[
                        self.rx_en.eq(0),
                    ]).Else([
                        self.rx_en.eq(self.rx_en | rx_en_delay),
                    ])
                ]).Else([
                    self.rx_en.eq(            self.reg10.fields.rx_en),
                ]),
            ]
        else:
            self.comb +=[
                self.tx_en.eq(            self.reg10.fields.tx_en),
                self.rx_en.eq(            self.reg10.fields.rx_en),
            ]

        # Stream delay signal mux
        tx_cases = {
            # 0: same as default value, comment for readability
            1: tx_en_delay.eq(self.tx_en_delay_signal[0]),
            2: tx_en_delay.eq(self.tx_en_delay_signal[1]),
            # Same behaviour as no delay if mode choice undefined
            "default": tx_en_delay.eq(1)
        }
        rx_cases = {
            # 0: same as default value, comment for readability
            1: rx_en_delay.eq(self.rx_en_delay_signal[0]),
            2: rx_en_delay.eq(self.rx_en_delay_signal[1]),
            # Same behaviour as no delay if mode choice undefined
            "default": rx_en_delay.eq(1)
        }
        self.comb += [
            Case(self.tx_en_delay_mode,tx_cases),
            Case(self.rx_en_delay_mode,rx_cases),
        ]
