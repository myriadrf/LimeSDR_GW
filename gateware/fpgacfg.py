#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

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
        self.wfm_ch_en         = CSRStorage(16, reset=0b11)
        self.wfm_smpl_width    = CSRStorage(2,  reset=0b10)

        # Peripheral config.
        self.spi_ss            = CSRStorage(16, reset=0xffff)
        self.clk_ena           = CSRStorage(4, reset=0b1111)              # 29

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

            self.from_fpgacfg.wfm_ch_en.eq(        self.wfm_ch_en.storage),
            self.from_fpgacfg.wfm_smpl_width.eq(   self.wfm_smpl_width.storage),

            # Peripheral config.
            self.from_fpgacfg.spi_ss.eq(           self.spi_ss.storage),

            self.from_fpgacfg.clk_ena.eq(          self.clk_ena.storage),
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
