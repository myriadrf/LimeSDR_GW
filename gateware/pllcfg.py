#
# This file is part of LiteX.
#
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
#
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from gateware.lms7002_top       import DelayControl

# PLL Cfg ------------------------------------------------------------------------------------------

class PLLCfg(LiteXModule):
    def __init__(self):
        self.delay_control    = DelayControl()
        self.auto_phcfg_smpls = Signal(16)

        self.reg01     = CSRStatus(16,  reset=1)
        self.pll_lock  = CSRStatus(16,  reset=0)
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
        self.cnt_phase = CSRStorage(16)
        self.reg05     = CSRStorage(16, fields=[
            CSRField("pllcfg_lf_cap", size=2, offset=0),
            CSRField("pllcfg_lf_res", size=5, offset=2),
            CSRField("pllcfg_vcodiv", size=1, offset=7),
            CSRField("chp_curr",      size=3, offset=8),
        ], reset=0x1F0)

        self.reg06     = CSRStorage(16, fields=[
            CSRField("n_byp",    size=1, offset=0),
            CSRField("n_odddiv", size=1, offset=1),
            CSRField("m_byp",    size=1, offset=2),
            CSRField("m_odddiv", size=1, offset=3),
        ], reset=0b1010)
        self.reg07     = CSRStorage(16, fields=[
            CSRField("c0_byp",    size=1, offset=0),
            CSRField("c0_odddiv", size=1, offset=1),
            CSRField("c1_byp",    size=1, offset=2),
            CSRField("c1_odddiv", size=1, offset=3),
            CSRField("c2_byp",    size=1, offset=4),
            CSRField("c2_odddiv", size=1, offset=5),
            CSRField("c3_byp",    size=1, offset=6),
            CSRField("c3_odddiv", size=1, offset=7),
            CSRField("c4_byp",    size=1, offset=8),
            CSRField("c4_odddiv", size=1, offset=9),
        ], reset=0xAAAA)

        self.n_cnt            = CSRStorage(16) # 10
        self.m_cnt            = CSRStorage(16) # 11
        self.c0_cnt           = CSRStorage(16) # 14
        self.c1_cnt           = CSRStorage(16)
        self.c2_cnt           = CSRStorage(16)
        self.c3_cnt           = CSRStorage(16)
        self.c4_cnt           = CSRStorage(16)
        self._auto_phcfg_smpls = CSRStorage(16, reset=0x3FF) # 30
        self.auto_phcfg_step  = CSRStorage(16, reset=0x002) # 31

        # # #

        # Logic.
        self.comb += [
            self.reg01.status.eq(Cat(1, 0, self.delay_control.done, self.delay_control.error, Constant(0, 12))),
            self.pll_lock.status.eq(    Constant(0, 16)),

            self.delay_control.en.eq(self.reg03.fields.phcfg_start),
            If(self.reg03.fields.cnt_ind == 0b0011,
                self.delay_control.sel.eq(0),
            ).Else(
                self.delay_control.sel.eq(3),
            ),
            self.delay_control.dir.eq( self.reg03.fields.phcfg_updn),
            self.delay_control.mode.eq(self.reg03.fields.phcfg_mode),
            self.auto_phcfg_smpls.eq(self._auto_phcfg_smpls.storage),
        ]
