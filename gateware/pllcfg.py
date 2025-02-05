#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# PLL Cfg ------------------------------------------------------------------------------------------

class PLLCfg(LiteXModule):
    def __init__(self):
        self.phcfg_error       = Signal()
        self.phcfg_done        = Signal()
        self.pllcfg_busy       = Signal()
        self.pllcfg_done       = Signal()

        self.pll_lock          = Signal(16)

        self.phcfg_tst         = Signal()
        self.phcfg_mode        = Signal()
        self.phcfg_updn        = Signal()
        self.cnt_ind           = Signal(5)
        self.pll_ind           = Signal(5)
        self.pllrst_start      = Signal()
        self.phcfg_start       = Signal()
        self.pllcfg_start      = Signal()

        self.cnt_phase         = Signal(16)

        self.chp_curr          = Signal(3)
        self.pllcfg_vcodiv     = Signal()
        self.pllcfg_lf_res     = Signal(5)
        self.pllcfg_lf_cap     = Signal(2)

        self.m_odddiv          = Signal()
        self.m_byp             = Signal()
        self.n_odddiv          = Signal()
        self.n_byp             = Signal()

        self.c0_byp            = Signal()
        self.c0_odddiv         = Signal()
        self.c1_byp            = Signal()
        self.c1_odddiv         = Signal()
        self.c2_byp            = Signal()
        self.c2_odddiv         = Signal()
        self.c3_byp            = Signal()
        self.c3_odddiv         = Signal()
        self.c4_byp            = Signal()
        self.c4_odddiv         = Signal()

        self.n_cnt             = Signal(16)
        self.m_cnt             = Signal(16)

        self.c0_cnt            = Signal(16)
        self.c1_cnt            = Signal(16)
        self.c2_cnt            = Signal(16)
        self.c3_cnt            = Signal(16)
        self.c4_cnt            = Signal(16)

        self.auto_phcfg_smpls  = Signal(16)
        self.auto_phcfg_step   = Signal(16)

        self.reg01             = CSRStatus(4,   reset=1)      # reg 01
        self._pll_lock         = CSRStatus(16,  reset=0)      # reg 02
        self.reg03             = CSRStorage(fields=[          # reg 03
            CSRField("pllcfg_start", size=1, offset=0,  reset=0),
            CSRField("phcfg_start",  size=1, offset=1,  reset=0),
            CSRField("pllrst_start", size=1, offset=2,  reset=0),
            CSRField("pll_ind",      size=5, offset=3,  reset=0),
            CSRField("cnt_ind",      size=5, offset=8,  reset=0),
            CSRField("phcfg_updn",   size=1, offset=13, reset=0),
            CSRField("phcfg_mode",   size=1, offset=14, reset=0),
            CSRField("phcfg_tst",    size=1, offset=15, reset=0),
        ])
        self._cnt_phase        = CSRStorage(16, reset=0)      # reg 04
        self.reg05             = CSRStorage(fields=[          # reg 05
            CSRField("pllcfg_lf_cap", size=2, offset=0, reset=0x00),
            CSRField("pllcfg_lf_res", size=5, offset=2, reset=0x1C),
            CSRField("pllcfg_vcodiv", size=1, offset=7, reset=0x03),
            CSRField("chp_curr",      size=3, offset=8, reset=0x01),
        ])
        self.reg06             = CSRStorage(fields=[          # reg 06
            CSRField("n_byp",    size=1, offset=0, reset=0),
            CSRField("n_odddiv", size=1, offset=1, reset=1),
            CSRField("m_byp",    size=1, offset=2, reset=0),
            CSRField("m_odddiv", size=1, offset=3, reset=1),
        ])
        self.reg07             = CSRStorage(fields=[          # reg 07
            CSRField("c0_byp",    size=1, offset=0, reset=0),
            CSRField("c0_odddiv", size=1, offset=1, reset=1),
            CSRField("c1_byp",    size=1, offset=2, reset=0),
            CSRField("c1_odddiv", size=1, offset=3, reset=1),
            CSRField("c2_byp",    size=1, offset=4, reset=0),
            CSRField("c2_odddiv", size=1, offset=5, reset=1),
            CSRField("c3_byp",    size=1, offset=6, reset=0),
            CSRField("c3_odddiv", size=1, offset=7, reset=1),
            CSRField("c4_byp",    size=1, offset=8, reset=0),
            CSRField("c4_odddiv", size=1, offset=9, reset=1),
        ])

        self._n_cnt            = CSRStorage(16, reset=0)      # reg 10
        self._m_cnt            = CSRStorage(16, reset=0)      # reg 11

        self._c0_cnt           = CSRStorage(16, reset=0)      # reg 14
        self._c1_cnt           = CSRStorage(16, reset=0)      # reg 15
        self._c2_cnt           = CSRStorage(16, reset=0)      # reg 16
        self._c3_cnt           = CSRStorage(16, reset=0)      # reg 17
        self._c4_cnt           = CSRStorage(16, reset=0)      # reg 18
        # reg 24-29: reserved for C10-C15 counters
        self._auto_phcfg_smpls = CSRStorage(16, reset=0xFFF)  # reg 30
        self._auto_phcfg_step  = CSRStorage(16, reset=0x002)  # reg 31

        # # #

        # Logic.
        self.comb += [
            self._pll_lock.status.eq(self.pll_lock),
            self.auto_phcfg_smpls.eq(self._auto_phcfg_smpls.storage),
            self.auto_phcfg_step.eq( self._auto_phcfg_step.storage),

            self.reg01.status.eq(Cat(self.pllcfg_done, self.pllcfg_busy, self.phcfg_done, self.phcfg_error)),

            self.phcfg_tst.eq(    self.reg03.fields.phcfg_tst),
            self.phcfg_mode.eq(   self.reg03.fields.phcfg_mode),
            self.phcfg_updn.eq(   self.reg03.fields.phcfg_updn),
            self.cnt_ind.eq(      self.reg03.fields.cnt_ind),
            self.pll_ind.eq(      self.reg03.fields.pll_ind),
            self.pllrst_start.eq( self.reg03.fields.pllrst_start),
            self.phcfg_start.eq(  self.reg03.fields.phcfg_start),
            self.pllcfg_start.eq( self.reg03.fields.pllcfg_start),

            self.cnt_phase.eq(    self._cnt_phase.storage),

            self.chp_curr.eq(     self.reg05.fields.chp_curr),
            self.pllcfg_vcodiv.eq(self.reg05.fields.pllcfg_vcodiv),
            self.pllcfg_lf_res.eq(self.reg05.fields.pllcfg_lf_res),
            self.pllcfg_lf_cap.eq(self.reg05.fields.pllcfg_lf_cap),

            self.m_odddiv.eq(     self.reg06.fields.m_odddiv),
            self.m_byp.eq(        self.reg06.fields.m_byp),
            self.n_odddiv.eq(     self.reg06.fields.n_odddiv),
            self.n_byp.eq(        self.reg06.fields.n_byp),

            self.c0_byp.eq(       self.reg07.fields.c0_byp),
            self.c0_odddiv.eq(    self.reg07.fields.c0_odddiv),
            self.c1_byp.eq(       self.reg07.fields.c1_byp),
            self.c1_odddiv.eq(    self.reg07.fields.c1_odddiv),
            self.c2_byp.eq(       self.reg07.fields.c2_byp),
            self.c2_odddiv.eq(    self.reg07.fields.c2_odddiv),
            self.c3_byp.eq(       self.reg07.fields.c3_byp),
            self.c3_odddiv.eq(    self.reg07.fields.c3_odddiv),
            self.c4_byp.eq(       self.reg07.fields.c4_byp),
            self.c4_odddiv.eq(    self.reg07.fields.c4_odddiv),

            self.n_cnt.eq(        self._n_cnt.storage),
            self.m_cnt.eq(        self._m_cnt.storage),

            self.c0_cnt.eq(       self._c0_cnt.storage),
            self.c1_cnt.eq(       self._c1_cnt.storage),
            self.c2_cnt.eq(       self._c2_cnt.storage),
            self.c3_cnt.eq(       self._c3_cnt.storage),
            self.c4_cnt.eq(       self._c4_cnt.storage),
        ]
