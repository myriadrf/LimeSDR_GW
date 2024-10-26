#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from gateware.common import FromFPGACfg

# Constants ----------------------------------------------------------------------------------------
MAJOR_REV           = 2
MINOR_REV           = 2
BETA_REV            = 1
COMPILE_REV         = 6
COMPILE_YEAR_STAMP  = 19
COMPILE_MONTH_STAMP = 5
COMPILE_DAY_STAMP   = 7
COMPILE_HOUR_STAMP  = 14

MAGIC_NUM           = 0xD8A5F009
BOARD_ID            = 0x0011 # LimeSDR-MINI

# FPGA Cfg -----------------------------------------------------------------------------------------

class FPGACfg(LiteXModule):
    def __init__(self, pads):
        self.from_fpgacfg      = FromFPGACfg()
        self.pwr_src           = Signal()
        self.led1_ctrl         = Signal(3)
        self.led2_ctrl         = Signal(3)
        self.led3_ctrl         = Signal(3)

        # Read only registers
        self.board_id          = CSRStatus(16,  reset=BOARD_ID)
        self.major_rev         = CSRStatus(16,  reset=MAJOR_REV)
        self.compile_rev       = CSRStatus(16,  reset=COMPILE_REV)
        self.bom_hw_ver        = CSRStatus(16,  reset=0)

        # FPGA direct clocking
        self.phase_reg_sel     = CSRStorage(16, reset=0)
        self.drct_clk_en       = CSRStorage(16, reset=0)
        # load_phase, cnt_int[4:0], clk_ind[4:0]
        self.load_phase        = CSRStorage(fields=[
            CSRField("clk_int",        size=5, offset=0),
            CSRField("cnt_int",        size=5, offset=5),
            CSRField("load_phase_reg", size=1, offset=10),
        ])

        # Interface config
        self.channel_cntrl     = CSRStorage(fields=[
            CSRField("ch_en", size=2, offset=0, reset=0b11, values=[
                ("``2b01", "Channel A"),
                ("``2b10", "Channel B"),
                ("``2b11", "Channels A and B")
            ])
        ])
        self.reg08             = CSRStorage(fields=[
            CSRField("smpl_width",  size=2, offset=0,  reset=0b10),
            CSRField("mode",        size=1, offset=5,  reset=0),
            CSRField("ddr_en",      size=1, offset=6,  reset=0),
            CSRField("trxiq_pulse", size=1, offset=7,  reset=0),
            CSRField("mimo_int_en", size=1, offset=8,  reset=1),
            CSRField("synch_dis",   size=1, offset=9,  reset=0),
            CSRField("synch_mode",  size=1, offset=10, reset=0),
        ])
        self.reg09             = CSRStorage(fields=[
            CSRField("smpl_nr_clr",    size=1, offset=0, reset=1),
            CSRField("txpct_loss_clr", size=1, offset=1, reset=1),
        ])
        self.reg10             = CSRStorage(fields=[
            CSRField("rx_en",       size=1, offset=0),
            CSRField("tx_en",       size=1, offset=1),
            CSRField("rx_ptrn_en" , size=1, offset=8),
            CSRField("tx_ptrn_en",  size=1, offset=9),
            CSRField("tx_cnt_en",   size=1, offset=10),
        ])
        self.wfm_ch_en         = CSRStorage(16, reset=0b11)
        self.reg13             = CSRStorage(fields=[
            CSRField("wfm_play", size=1, offset=1),
            CSRField("wfm_load", size=1, offset=2),
        ])
        self.wfm_smpl_width    = CSRStorage(2,  reset=0b10)
        self.sync_size         = CSRStorage(16, reset=0x03FC)

        # Peripheral config.
        self.txant_pre         = CSRStorage(16, reset=1)
        self.txant_post        = CSRStorage(16, reset=1)

        self.SPI_SS            = CSRStorage(16, reset=0xffff)
        self.LMS1              = CSRStorage(fields=[         # 19
            CSRField("SS",          size=1, offset=0, reset=1),
            CSRField("RESET",       size=1, offset=1, reset=1),
            CSRField("CORE_LDO_EN", size=1, offset=2, reset=0),
            CSRField("TXNRX1",      size=1, offset=3, reset=1),
            CSRField("TXNRX2",      size=1, offset=4, reset=0),
            CSRField("TXEN",        size=1, offset=5, reset=1),
            CSRField("RXEN",        size=1, offset=6, reset=1),
        ])
        self.GPIO              = CSRStorage(16, reset=0b0001000101000100) # 23
        self.fpga_led_ctrl     = CSRStorage(fields=[                      # 26
            CSRField("LED1_CTRL", size=3, offset=0),
            CSRField("LED2_CTRL", size=3, offset=4),
        ])
        self.FX3_LED_CTRL      = CSRStorage(3)                            # 28
        self.CLK_ENA           = CSRStorage(4, reset=0b1111)              # 29
        self.sync_pulse_period = CSRStorage(32, reset=0x3D090)            # 30

        # # #

        # Signals.
        _bom_ver = Signal(3)
        _hw_ver  = Signal(4)

        # Logic.
        self.comb += [
            self.bom_hw_ver.status.eq(             Cat(_hw_ver, _bom_ver, self.pwr_src, Constant(0, 8))),
            # FPGA direct clocking
            self.from_fpgacfg.phase_reg_sel.eq(    self.phase_reg_sel.storage),
            self.from_fpgacfg.drct_clk_en.eq(      self.drct_clk_en.storage),
            self.from_fpgacfg.load_phase_reg.eq(   self.load_phase.fields.load_phase_reg),
            self.from_fpgacfg.clk_ind.eq(          self.load_phase.fields.clk_int),
            self.from_fpgacfg.cnt_ind.eq(          self.load_phase.fields.cnt_int),
            self.from_fpgacfg.ch_en.eq(            self.channel_cntrl.storage),

            self.from_fpgacfg.smpl_width.eq(       self.reg08.fields.smpl_width),
            self.from_fpgacfg.mode.eq(             self.reg08.fields.mode),
            self.from_fpgacfg.ddr_en.eq(           self.reg08.fields.ddr_en),
            self.from_fpgacfg.trxiq_pulse.eq(      self.reg08.fields.trxiq_pulse),
            self.from_fpgacfg.mimo_int_en.eq(      self.reg08.fields.mimo_int_en),
            self.from_fpgacfg.synch_dis.eq(        self.reg08.fields.synch_dis),
            self.from_fpgacfg.synch_mode.eq(       self.reg08.fields.synch_mode),

            self.from_fpgacfg.smpl_nr_clr.eq(      self.reg09.fields.smpl_nr_clr),
            self.from_fpgacfg.txpct_loss_clr.eq(   self.reg09.fields.txpct_loss_clr),

            self.from_fpgacfg.rx_en.eq(            self.reg10.fields.rx_en),
            self.from_fpgacfg.tx_en.eq(            self.reg10.fields.tx_en),
            self.from_fpgacfg.rx_ptrn_en.eq(       self.reg10.fields.rx_ptrn_en),
            self.from_fpgacfg.tx_ptrn_en.eq(       self.reg10.fields.tx_ptrn_en),
            self.from_fpgacfg.tx_cnt_en.eq(        self.reg10.fields.tx_cnt_en),

            self.from_fpgacfg.wfm_ch_en.eq(        self.wfm_ch_en.storage),
            self.from_fpgacfg.wfm_play.eq(         self.reg13.fields.wfm_play),
            self.from_fpgacfg.wfm_load.eq(         self.reg13.fields.wfm_load),
            self.from_fpgacfg.wfm_smpl_width.eq(   self.wfm_smpl_width.storage),
            self.from_fpgacfg.sync_size.eq(        self.sync_size.storage),

            # Peripheral config.
            self.from_fpgacfg.txant_pre.eq(        self.txant_pre.storage),
            self.from_fpgacfg.txant_post.eq(       self.txant_post.storage),

            self.from_fpgacfg.SPI_SS.eq(           self.SPI_SS.storage),

            self.from_fpgacfg.LMS1_SS.eq(          self.LMS1.fields.SS),
            self.from_fpgacfg.LMS1_RESET.eq(       self.LMS1.fields.RESET),
            self.from_fpgacfg.LMS1_CORE_LDO_EN.eq( self.LMS1.fields.CORE_LDO_EN),
            self.from_fpgacfg.LMS1_TXNRX1.eq(      self.LMS1.fields.TXNRX1),
            self.from_fpgacfg.LMS1_TXNRX2.eq(      self.LMS1.fields.TXNRX2),
            self.from_fpgacfg.LMS1_TXEN.eq(        self.LMS1.fields.TXEN),
            self.from_fpgacfg.LMS1_RXEN.eq(        self.LMS1.fields.RXEN),

            self.from_fpgacfg.GPIO.eq(             self.GPIO.storage),
            self.from_fpgacfg.FPGA_LED1_CTRL.eq(   self.fpga_led_ctrl.fields.LED1_CTRL),
            self.from_fpgacfg.FPGA_LED2_CTRL.eq(   self.fpga_led_ctrl.fields.LED2_CTRL),
            self.from_fpgacfg.FX3_LED_CTRL.eq(     self.FX3_LED_CTRL.storage),
            self.from_fpgacfg.CLK_ENA.eq(          self.CLK_ENA.storage),
            self.from_fpgacfg.sync_pulse_period.eq(self.sync_pulse_period.storage),
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
