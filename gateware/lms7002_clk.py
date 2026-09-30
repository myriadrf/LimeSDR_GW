#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import EventManager, EventSourceProcess
from litex.soc.cores.clock import *

from litescope import LiteScopeAnalyzer

# Clk Cfg Regs -------------------------------------------------------------------------------------

class _StorageProxy:
    """Transparent proxy exposing legacy .storage and .re attributes for unified CSR fields."""
    def __init__(self, storage_sig, parent_csr=None):
        self.storage = storage_sig
        self._parent_csr = parent_csr

    @property
    def status(self):
        return self.storage

    @property
    def re(self):
        if self._parent_csr is not None:
            return self._parent_csr.re
        return Signal()


class _StatusProxy:
    """Transparent proxy exposing legacy .status and .storage attributes for unified CSR fields."""
    def __init__(self, status_sig):
        self.status = status_sig

    @property
    def storage(self):
        return self.status


class ClkCfgRegs(LiteXModule):
    def __init__(self, use_status_regs=False, unified_csr=False):
        if unified_csr:
            # -------------------------------------------------------------------------------------
            # Unified Clock Configuration CSR Architecture
            # -------------------------------------------------------------------------------------
            # Small 1-bit registers are unified into semantic functional registers matching the
            # 16-bit LMS64C protocol register bit allocations. This significantly reduces Wishbone
            # decode logic, write strobes, and read multiplexer inputs on Cyclone IV FPGAs.

            # LMS64C Register 0x0005: Direct Clock Control
            # Bits:
            #   [0] DRCT_TXCLK_EN: TX CLK source selection (0: PLL, 1: Direct clock)
            #   [1] DRCT_RXCLK_EN: RX CLK source selection (0: PLL, 1: Direct clock)
            self.drct_clk_ctrl = CSRStorage(fields=[
                CSRField("drct_txclk_en", size=1, offset=0, reset=0,
                         description="TX CLK source selection: 0: PLL, 1: Direct clock"),
                CSRField("drct_rxclk_en", size=1, offset=1, reset=0,
                         description="RX CLK source selection: 0: PLL, 1: Direct clock"),
            ], description="Direct clock control (LMS64C 0x0005)")

            # LMS64C Register 0x0020: C1 Phase Offset
            # Bits [8:0]: C1 phase offset in degrees
            self.c1_phase = CSRStorage(size=9, reset=0,
                                       description="Clock output 1 phase offset, in degrees (LMS64C 0x0020)")

            # LMS64C Register 0x0021: PLL & Phase Configuration Status
            # Bits:
            #   [0] PLLCFG_DONE:  PLL configuration done (0: Not done, 1: Done)
            #   [1] PLLCFG_BUSY:  PLL configuration busy (0: Idle, 1: Busy)
            #   [2] PHCFG_DONE:   Phase configuration done (0: Not done, 1: Done)
            #   [3] PHCFG_ERR:    Phase configuration error (0: No error, 1: Error)
            #   [7] PLLCFG_ERROR: PLL configuration error (0: No error, 1: Error)
            self.pll_status = CSRStatus(fields=[
                CSRField("pllcfg_done",  size=1, offset=0, reset=0,
                         description="PLL configuration done: 0: Not done, 1: Done"),
                CSRField("pllcfg_busy",  size=1, offset=1, reset=0,
                         description="Clock config busy: 0: Idle, 1: Busy"),
                CSRField("phcfg_done",   size=1, offset=2, reset=0,
                         description="Phase config done: 0: Not done, 1: Done"),
                CSRField("phcfg_err",    size=1, offset=3, reset=0,
                         description="Phase config error: 0: No error, 1: Error"),
                CSRField("pllcfg_error", size=1, offset=7, reset=0,
                         description="PLL configuration error: 0: No error, 1: Error"),
            ], description="PLL and phase configuration status (LMS64C 0x0021)")

            # LMS64C Register 0x0022: PLL Lock Status
            # Bits [15:0]: Array of PLL locked flags (0: Not locked, 1: Locked)
            self.pll_lock = CSRStatus(size=16, reset=0,
                                      description="PLL lock status array: 0: Not locked, 1: Locked (LMS64C 0x0022)")

            # LMS64C Register 0x0023: PLL & Phase Configuration Control
            # Bits:
            #   [0]    PLLCFG_START: Start PLL configuration (0 to 1 transition)
            #   [1]    PHCFG_START:  Start phase configuration (0 to 1 transition)
            #   [2]    PLLRST_START: Start PLL reset (0 to 1 transition)
            #   [7:3]  PLL_IND:      PLL index for reconfiguration
            #   [12:8] CNT_IND:      Counter index for reconfiguration (0: All, 1: M, 2: C0, 3: C1, etc.)
            #   [13]   PHCFG_UPDN:   Phase shift direction (0: Down, 1: Up)
            #   [14]   PHCFG_MODE:   Phase configuration mode (0: Manual, 1: Auto)
            self.pll_ctrl = CSRStorage(fields=[
                CSRField("pllcfg_start", size=1, offset=0,  reset=0,
                         description="Start PLL configuration: 0 to 1 transition"),
                CSRField("phcfg_start",  size=1, offset=1,  reset=0,
                         description="Start phase configuration: 0 to 1 transition"),
                CSRField("pllrst_start", size=1, offset=2,  reset=0,
                         description="Start PLL reset: 0 to 1 transition"),
                CSRField("pll_ind",      size=5, offset=3,  reset=0,
                         description="PLL index for reconfiguration"),
                CSRField("cnt_ind",      size=5, offset=8,  reset=0,
                         description="Counter index for reconfiguration"),
                CSRField("phcfg_updn",   size=1, offset=13, reset=0,
                         description="Phase shift direction: 0: Down, 1: Up"),
                CSRField("phcfg_mode",   size=1, offset=14, reset=0,
                         description="Phase configuration mode: 0: Manual, 1: Auto"),
            ], description="PLL and phase configuration control (LMS64C 0x0023)")

            # LMS64C Register 0x0024: Counter Phase Value
            # Bits [15:0]: Phase step counter value
            self.cnt_phase = CSRStorage(size=16, reset=0,
                                        description="Counter phase value (LMS64C 0x0024)")

            # LMS64C Register 0x0025: PLL VCO Divider Control
            # Bits:
            #   [7] PLLCFG_VCODIV: PLL VCO divider (0: disabled, 1: enabled)
            self.pll_vcodiv = CSRStorage(fields=[
                CSRField("pllcfg_vcodiv", size=1, offset=7, reset=0,
                         description="PLL VCO divider: 0: 0, 1: 1"),
            ], description="PLL VCO divider control (LMS64C 0x0025)")

            # LMS64C Register 0x0026: M and N Counter Divider Control
            # Bits:
            #   [0] N_DIV_BYP: N counter divider bypass (0: Normal, 1: Bypass)
            #   [1] N_ODD_DIV: N counter odd divider (0: Even, 1: Odd)
            #   [2] M_DIV_BYP: M counter divider bypass (0: Normal, 1: Bypass)
            #   [3] M_ODD_DIV: M counter odd divider (0: Even, 1: Odd)
            self.mn_div_ctrl = CSRStorage(fields=[
                CSRField("n_div_byp", size=1, offset=0, reset=0,
                         description="N counter divider bypass: 0: normal, 1: bypass"),
                CSRField("n_odd_div", size=1, offset=1, reset=1,
                         description="N counter odd divider: 0: even, 1: odd"),
                CSRField("m_div_byp", size=1, offset=2, reset=0,
                         description="M counter divider bypass: 0: normal, 1: bypass"),
                CSRField("m_odd_div", size=1, offset=3, reset=1,
                         description="M counter odd divider: 0: even, 1: odd"),
            ], description="M and N counter divider control (LMS64C 0x0026)")

            # LMS64C Register 0x0027: C0-C4 Output Divider Control
            # Bits:
            #   [0] C0_DIV_BYP: Clock output 0 divider bypass (0: Normal, 1: Bypass)
            #   [1] C0_ODDDIV:  Clock output 0 odd divider (0: Even, 1: Odd)
            #   [2] C1_DIV_BYP: Clock output 1 divider bypass (0: Normal, 1: Bypass)
            #   [3] C1_ODDDIV:  Clock output 1 odd divider (0: Even, 1: Odd)
            #   [4] C2_DIV_BYP: Clock output 2 divider bypass (0: Normal, 1: Bypass)
            #   [5] C2_ODDDIV:  Clock output 2 odd divider (0: Even, 1: Odd)
            #   [6] C3_DIV_BYP: Clock output 3 divider bypass (0: Normal, 1: Bypass)
            #   [7] C3_ODDDIV:  Clock output 3 odd divider (0: Even, 1: Odd)
            #   [8] C4_DIV_BYP: Clock output 4 divider bypass (0: Normal, 1: Bypass)
            #   [9] C4_ODDDIV:  Clock output 4 odd divider (0: Even, 1: Odd)
            self.c_div_ctrl = CSRStorage(fields=[
                CSRField("c0_div_byp", size=1, offset=0, reset=0,
                         description="Clock output 0 divider bypass: 0: do not bypass, 1: bypass"),
                CSRField("c0_odddiv",  size=1, offset=1, reset=1,
                         description="Clock output 0 odd divider: 0: even, 1: odd"),
                CSRField("c1_div_byp", size=1, offset=2, reset=0,
                         description="Clock output 1 divider bypass: 0: do not bypass, 1: bypass"),
                CSRField("c1_odddiv",  size=1, offset=3, reset=1,
                         description="Clock output 1 odd divider: 0: even, 1: odd"),
                CSRField("c2_div_byp", size=1, offset=4, reset=0,
                         description="Clock output 2 divider bypass: 0: do not bypass, 1: bypass"),
                CSRField("c2_odddiv",  size=1, offset=5, reset=1,
                         description="Clock output 2 odd divider: 0: even, 1: odd"),
                CSRField("c3_div_byp", size=1, offset=6, reset=0,
                         description="Clock output 3 divider bypass: 0: do not bypass, 1: bypass"),
                CSRField("c3_odddiv",  size=1, offset=7, reset=1,
                         description="Clock output 3 odd divider: 0: even, 1: odd"),
                CSRField("c4_div_byp", size=1, offset=8, reset=0,
                         description="Clock output 4 divider bypass: 0: do not bypass, 1: bypass"),
                CSRField("c4_odddiv",  size=1, offset=9, reset=1,
                         description="Clock output 4 odd divider: 0: even, 1: odd"),
            ], description="C0-C4 clock output divider control (LMS64C 0x0027)")

            # LMS64C Register 0x002A: N Counter Value
            self.n_cnt = CSRStorage(size=16, reset=0,
                                    description="PLL N counter values (LMS64C 0x002A)")

            # LMS64C Register 0x002B: M Counter Value
            self.m_cnt = CSRStorage(size=16, reset=0,
                                    description="PLL M counter values (LMS64C 0x002B)")

            # LMS64C Register 0x002E: C0 Divider Counter Value
            self.c0_div_cnt = CSRStorage(size=16, reset=0,
                                         description="Clock output 0 divider counter values (LMS64C 0x002E)")

            # LMS64C Register 0x002F: C1 Divider Counter Value
            self.c1_div_cnt = CSRStorage(size=16, reset=0,
                                         description="Clock output 1 divider counter values (LMS64C 0x002F)")

            # LMS64C Register 0x0030: C2 Divider Counter Value
            self.c2_div_cnt = CSRStorage(size=16, reset=0,
                                         description="Clock output 2 divider counter values (LMS64C 0x0030)")

            # LMS64C Register 0x0031: C3 Divider Counter Value
            self.c3_div_cnt = CSRStorage(size=16, reset=0,
                                         description="Clock output 3 divider counter values (LMS64C 0x0031)")

            # LMS64C Register 0x0032: C4 Divider Counter Value
            self.c4_div_cnt = CSRStorage(size=16, reset=0,
                                         description="Clock output 4 divider counter values (LMS64C 0x0032)")

            # LMS64C Register 0x003E: Auto Phase Configuration Samples
            self.auto_phcfg_smpls = CSRStorage(size=16, reset=0xEFFF,
                                               description="Number of samples to use during auto phase configuration (LMS64C 0x003E)")

            # LMS64C Register 0x003F: Auto Phase Configuration Step
            self.auto_phcfg_step = CSRStorage(size=16, reset=0x002,
                                              description="Phase configuration step size (LMS64C 0x003F)")

            # -------------------------------------------------------------------------------------
            # Transparent Proxies for Legacy Gateware Wiring
            # -------------------------------------------------------------------------------------
            # Modules such as LMS7002CLK_Altera and LimeTop connect directly to legacy attribute
            # names (.storage, .status, .re). These proxies forward access to the unified registers
            # without generating duplicate LiteX CSRs.
            self.DRCT_TXCLK_EN    = _StorageProxy(self.drct_clk_ctrl.fields.drct_txclk_en, self.drct_clk_ctrl)
            self.DRCT_RXCLK_EN    = _StorageProxy(self.drct_clk_ctrl.fields.drct_rxclk_en, self.drct_clk_ctrl)
            self.PHCFG_MODE       = _StorageProxy(self.pll_ctrl.fields.phcfg_mode, self.pll_ctrl)
            self.PHCFG_UPDN       = _StorageProxy(self.pll_ctrl.fields.phcfg_updn, self.pll_ctrl)
            self.PHCFG_DONE       = _StatusProxy(self.pll_status.fields.phcfg_done)
            self.PHCFG_ERR        = _StatusProxy(self.pll_status.fields.phcfg_err)
            self.PLLCFG_DONE      = _StatusProxy(self.pll_status.fields.pllcfg_done)
            self.PLLCFG_BUSY      = _StatusProxy(self.pll_status.fields.pllcfg_busy)
            self.PLLCFG_START     = _StorageProxy(self.pll_ctrl.fields.pllcfg_start, self.pll_ctrl)
            self.CNT_PHASE        = _StorageProxy(self.cnt_phase.storage, self.cnt_phase)
            self.PLLCFG_VCODIV    = _StorageProxy(self.pll_vcodiv.fields.pllcfg_vcodiv, self.pll_vcodiv)
            self.M_ODD_DIV        = _StorageProxy(self.mn_div_ctrl.fields.m_odd_div, self.mn_div_ctrl)
            self.M_Div_BYP        = _StorageProxy(self.mn_div_ctrl.fields.m_div_byp, self.mn_div_ctrl)
            self.N_ODD_DIV        = _StorageProxy(self.mn_div_ctrl.fields.n_odd_div, self.mn_div_ctrl)
            self.N_Div_BYP        = _StorageProxy(self.mn_div_ctrl.fields.n_div_byp, self.mn_div_ctrl)
            self.PLLRST_START     = _StorageProxy(self.pll_ctrl.fields.pllrst_start, self.pll_ctrl)
            self.PLL_IND          = _StorageProxy(self.pll_ctrl.fields.pll_ind, self.pll_ctrl)
            self.CNT_IND          = _StorageProxy(self.pll_ctrl.fields.cnt_ind, self.pll_ctrl)
            self.PLL_LOCK         = _StatusProxy(self.pll_lock.status)
            self.PHCFG_START      = _StorageProxy(self.pll_ctrl.fields.phcfg_start, self.pll_ctrl)
            self.PLLCFG_ERROR     = _StatusProxy(self.pll_status.fields.pllcfg_error)
            self.VCO_Mult_BYP     = _StorageProxy(Signal(1))
            self.VCO_Div_BYP      = _StorageProxy(Signal(1))
            self.C0_Div_BYP       = _StorageProxy(self.c_div_ctrl.fields.c0_div_byp, self.c_div_ctrl)
            self.C1_Div_BYP       = _StorageProxy(self.c_div_ctrl.fields.c1_div_byp, self.c_div_ctrl)
            self.C2_Div_BYP       = _StorageProxy(self.c_div_ctrl.fields.c2_div_byp, self.c_div_ctrl)
            self.C3_Div_BYP       = _StorageProxy(self.c_div_ctrl.fields.c3_div_byp, self.c_div_ctrl)
            self.C4_Div_BYP       = _StorageProxy(self.c_div_ctrl.fields.c4_div_byp, self.c_div_ctrl)
            self.C0_ODDDIV        = _StorageProxy(self.c_div_ctrl.fields.c0_odddiv, self.c_div_ctrl)
            self.C1_ODDDIV        = _StorageProxy(self.c_div_ctrl.fields.c1_odddiv, self.c_div_ctrl)
            self.C2_ODDDIV        = _StorageProxy(self.c_div_ctrl.fields.c2_odddiv, self.c_div_ctrl)
            self.C3_ODDDIV        = _StorageProxy(self.c_div_ctrl.fields.c3_odddiv, self.c_div_ctrl)
            self.C4_ODDDIV        = _StorageProxy(self.c_div_ctrl.fields.c4_odddiv, self.c_div_ctrl)
            self.N_CNT            = _StorageProxy(self.n_cnt.storage, self.n_cnt)
            self.M_CNT            = _StorageProxy(self.m_cnt.storage, self.m_cnt)
            self.VCO_Div_CNT      = _StorageProxy(Signal(16))
            self.VCO_Mult_CNT     = _StorageProxy(Signal(16))
            self.C0_Div_CNT       = _StorageProxy(self.c0_div_cnt.storage, self.c0_div_cnt)
            self.C1_Div_CNT       = _StorageProxy(self.c1_div_cnt.storage, self.c1_div_cnt)
            self.C2_Div_CNT       = _StorageProxy(self.c2_div_cnt.storage, self.c2_div_cnt)
            self.C3_Div_CNT       = _StorageProxy(self.c3_div_cnt.storage, self.c3_div_cnt)
            self.C4_Div_CNT       = _StorageProxy(self.c4_div_cnt.storage, self.c4_div_cnt)
            self.C1_Phase         = _StorageProxy(self.c1_phase.storage, self.c1_phase)
            self.Auto_PHcfg_smpls = _StorageProxy(self.auto_phcfg_smpls.storage, self.auto_phcfg_smpls)
            self.Auto_PHcfg_step  = _StorageProxy(self.auto_phcfg_step.storage, self.auto_phcfg_step)

        else:
            # --------- Legacy Clocking CFG registers (disintegrated 1-bit CSRs) ----------------------
            # Control registers
            self.DRCT_TXCLK_EN = CSRStorage(size=1, reset=0,
                                         description="TX CLK source selection: 0: PLL, 1: Direct clock")
            self.DRCT_RXCLK_EN = CSRStorage(size=1, reset=0,
                                         description="RX CLK source selection: 0: PLL, 1: Direct clock")
            self.PHCFG_MODE = CSRStorage(size=1, reset=0,
                                         description="Phase configuration mode: 0: Manual, 1: Auto")
            self.PHCFG_UPDN = CSRStorage(size=1, reset=0,
                                 description="Phase shift direction : 0: Down, 1: Up")
            if use_status_regs:
                self.PHCFG_DONE = CSRStatus(size=1, reset=0,
                                             description="Phase config done: 0: Not done, 1: Done  ")
                self.PHCFG_ERR  = CSRStatus(size=1, reset=0,
                                             description="Phase config error: 0: no error, 1: error")
                self.PLLCFG_DONE = CSRStatus(size=1, reset=0,
                                              description="PLL configuration done: 0: Not done, 1: Done")
                self.PLLCFG_BUSY = CSRStatus(size=1, reset=0,
                                              description="Clock config busy: 0: Idle, 1: Busy")
            else:
                self.PHCFG_DONE = CSRStorage(size=1, reset=0,
                                             description="Phase config done: 0: Not done, 1: Done  ")
                self.PHCFG_ERR  = CSRStorage(size=1, reset=0,
                                             description="Phase config error: 0: no error, 1: error")
                self.PLLCFG_DONE = CSRStorage(size=1, reset=0,
                                              description="PLL configuration done: 0: Not done, 1: Done")
                self.PLLCFG_BUSY = CSRStorage(size=1, reset=0,
                                              description="Clock config busy: 0: Idle, 1: Busy")
            self.PLLCFG_START = CSRStorage(size=1, reset=0,
                                           description="Start PLL configuration: 0: idle, 0 to 1 transition: start configuration")
            self.CNT_PHASE = CSRStorage(size=16, reset=0,
                                        description="Counter phase value")
            self.PLLCFG_VCODIV = CSRStorage(size=1, reset=0,
                                            description="PLL VCO divider: 0: 0, 1: 1")
            self.M_ODD_DIV = CSRStorage(size=1, reset=0,
                                            description="M counter odd divider: 0: even, 1: odd")
            self.M_Div_BYP = CSRStorage(size=1, reset=0,
                                            description="M counter divider bypass: 0: normal, 1: bypass")
            self.N_ODD_DIV = CSRStorage(size=1, reset=0,
                                            description="N counter odd divider: 0: even, 1: odd")
            self.N_Div_BYP = CSRStorage(size=1, reset=0,
                                            description="N counter divider bypass: 0: normal, 1: bypass")
            self.PLLRST_START = CSRStorage(size=1, reset=0,
                                           description="Start PLL reset: 0: idle, 0 to 1 transition: start configuration")
            self.PLL_IND = CSRStorage(size=5, reset=0,
                                      description="PLL index for reconfiguration")
            self.CNT_IND = CSRStorage(size=5, reset=0,
                                      description="Counter index for reconfiguration: 0: All counters, 1 - M counter, 2 - C0 counter, 3 - C1 counter")
            self.PLL_LOCK = CSRStatus(size=16, reset=0,
                                          description="PLL lock status array: 0: not locked, 1: locked")
            self.PHCFG_START = CSRStorage(size=1, reset=0,
                                          description="Start phase configuration: 0: idle, 0 to 1 transition: start configuration")
            self.PLLCFG_ERROR = CSRStorage(size=1, reset=0,
                                           description="PLL configuration error: 0: no error, 1: error")
            self.VCO_Mult_BYP = CSRStorage(size=1, reset=0,
                                           description="PLL multiplier bypass: 0: do not bypass, 1: bypass")
            self.VCO_Div_BYP = CSRStorage(size=1, reset=0,
                                          description="PLL divider bypass: 0: do not bypass, 1: bypass")
            self.C0_Div_BYP = CSRStorage(size=1, reset=0,
                                         description="Clock output 0 divider bypass: 0: do not bypass, 1: bypass")
            self.C1_Div_BYP = CSRStorage(size=1, reset=0,
                                         description="Clock output 1 divider bypass: 0: do not bypass, 1: bypass")
            self.C2_Div_BYP = CSRStorage(size=1, reset=0,
                                         description="Clock output 2 divider bypass: 0: do not bypass, 1: bypass")
            self.C3_Div_BYP = CSRStorage(size=1, reset=0,
                                         description="Clock output 3 divider bypass: 0: do not bypass, 1: bypass")
            self.C4_Div_BYP = CSRStorage(size=1, reset=0,
                                         description="Clock output 4 divider bypass: 0: do not bypass, 1: bypass")
            self.C0_ODDDIV = CSRStorage(size=1, reset=0,
                                        description="Clock output 0 odd divider: 0: even, 1: odd")
            self.C1_ODDDIV = CSRStorage(size=1, reset=0,
                                        description="Clock output 1 odd divider: 0: even, 1: odd")
            self.C2_ODDDIV = CSRStorage(size=1, reset=0,
                                        description="Clock output 2 odd divider: 0: even, 1: odd")
            self.C3_ODDDIV = CSRStorage(size=1, reset=0,
                                        description="Clock output 3 odd divider: 0: even, 1: odd")
            self.C4_ODDDIV = CSRStorage(size=1, reset=0,
                                        description="Clock output 4 odd divider: 0: even, 1: odd")
            self.N_CNT = CSRStorage(size=16, reset=0,
                                    description="PLL N counter values")
            self.M_CNT = CSRStorage(size=16, reset=0,
                                    description="PLL M counter values")
            self.VCO_Div_CNT = CSRStorage(size=16, reset=0,
                                          description="PLL VCO divider counter values")
            self.VCO_Mult_CNT = CSRStorage(size=16, reset=0,
                                           description="PLL VCO multiplier counter values")
            self.C0_Div_CNT = CSRStorage(size=16, reset=0,
                                         description="Clock output 0 divider counter values")
            self.C1_Div_CNT = CSRStorage(size=16, reset=0,
                                         description="Clock output 1 divider counter values")
            self.C2_Div_CNT = CSRStorage(size=16, reset=0,
                                         description="Clock output 2 divider counter values")
            self.C3_Div_CNT = CSRStorage(size=16, reset=0,
                                         description="Clock output 3 divider counter values")
            self.C4_Div_CNT = CSRStorage(size=16, reset=0,
                                         description="Clock output 4 divider counter values")
            self.C1_Phase = CSRStorage(size=9, reset=0,
                                       description="Clock output 1 phase offset, in degrees")
            self.Auto_PHcfg_smpls = CSRStorage(size=16, reset=0xEFFF,
                                               description="Number of samples to use during auto phase configuration")
            self.Auto_PHcfg_step = CSRStorage(size=16, reset=0x002,description="Phase configuration step size")

# Xilinx LMS MM-------------------------------------------------------------------------------------

class XilinxLmsMMCM(LiteXModule):
    def __init__(self, platform, speedgrade, max_freq, mclk, fclk, logic_cd):
        platform.add_period_constraint(mclk, 1e9 / max_freq)

        self.cd_clkout0 = ClockDomain()

        self.mmcm = S7MMCM(speedgrade)
        self.mmcm.csr_reset = CSRStorage(reset=0, reset_less=True)
        self.mmcm.register_clkin(mclk, max_freq)
        self.mmcm.create_clkout(cd=self.cd_clkout0, freq=max_freq, buf=None)
        self.mmcm.create_clkout(cd=logic_cd, freq=max_freq, buf=None)
        self.mmcm.expose_drp()

        self.comb += self.mmcm.reset.eq(self.mmcm.csr_reset.storage)

        # DRP DRDY overrides
        drdy_signal = Signal()
        self.mmcm.latched_drdy = CSRStatus()
        self.mmcm.latched_drdy_reset = CSRStorage(reset=0)
        # drdy signal in mmcm class is local, thus to access the drdy outside the class
        # we need to override the port assignment
        self.mmcm.params.update(
            o_DRDY=drdy_signal
        )
        # Implement the drdy latching
        self.mmcm.sync += [
            If(self.mmcm.latched_drdy_reset.storage == 1,
                self.mmcm.latched_drdy.status.eq(0)
            ).Elif(drdy_signal,
                self.mmcm.latched_drdy.status.eq(1)
            )
        ]

        self.comb += fclk.eq(self.cd_clkout0.clk)

# Clk Mux ------------------------------------------------------------------------------------------

class ClkMux(LiteXModule):
    def __init__(self, i0, i1, o, sel):

        self.specials += Instance("BUFGCTRL",
            p_INIT_OUT      = 0,
            p_PRESELECT_I0  = False,
            p_PRESELECT_I1  = False,
            o_O         = o,
            i_CE0       = 1,
            i_CE1       = 1,
            i_I0        = i0,
            i_I1        = i1,
            i_IGNORE0   = 1,
            i_IGNORE1   = 1,
            i_S0        = ~sel,
            i_S1        = sel,
        )

# Clk Dly Fxd --------------------------------------------------------------------------------------

class ClkDlyFxd(LiteXModule):
    def __init__(self, i, o, dly_val=31, refclk_freq=200):

        self.specials += Instance("IDELAYE2",
            p_CINVCTRL_SEL              = True,
            p_DELAY_SRC                 = "DATAIN",
            p_HIGH_PERFORMANCE_MODE     = True,
            p_IDELAY_TYPE               = "FIXED",
            p_IDELAY_VALUE              = dly_val,
            p_PIPE_SEL                  = False,
            p_REFCLK_FREQUENCY          = refclk_freq,
            p_SIGNAL_PATTERN            = "CLOCK",
            o_CNTVALUEOUT   = None,
            o_DATAOUT       = o,
            i_C             = 0,
            i_CE            = 0,
            i_CINVCTRL      = 0,
            i_CNTVALUEIN    = 0,
            i_DATAIN        = i,
            i_IDATAIN       = 0,
            i_INC           = 0,
            i_LD            = 0,
            i_LDPIPEEN      = 0,
            i_REGRST        = 0
        )
