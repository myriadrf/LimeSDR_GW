#!/usr/bin/env python3

from migen import *

from litex.gen import *

from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import EventManager, EventSourceProcess
from litex.soc.cores.clock import *

from litescope import LiteScopeAnalyzer


class ClkCfgRegs(LiteXModule):
    def __init__(self):
        # --------- Clocking CFG registers --------------------------------------------------------
        # Control registers
        self.DRCT_TXCLK_EN = CSRStorage(size=1, reset=0,
                                     description="TX CLK source selection: 0: PLL, 1: Direct clock")
        self.DRCT_RXCLK_EN = CSRStorage(size=1, reset=0,
                                     description="RX CLK source selection: 0: PLL, 1: Direct clock")
        self.PHCFG_MODE = CSRStorage(size=1, reset=0,
                                     description="Phase configuration mode: 0: Manual, 1: Auto")
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
        self.PLLRST_START = CSRStorage(size=1, reset=0,
                                       description="Start PLL reset: 0: idle, 0 to 1 transition: start configuration")
        self.PLL_IND = CSRStorage(size=1, reset=0,
                                  description="PLL/MMCM index for reconfiguration: 0: TX PLL, 1: RX PLL")
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
        self.VCO_Div_CNT = CSRStorage(size=6, reset=0,
                                      description="PLL VCO divider value")
        self.VCO_Mult_CNT = CSRStorage(size=6, reset=0,
                                       description="PLL VCO multiplier value")
        self.C0_Div_CNT = CSRStorage(size=6, reset=0,
                                     description="Clock output 0 divider value")
        self.C1_Div_CNT = CSRStorage(size=6, reset=0,
                                     description="Clock output 1 divider value")
        self.C1_Phase = CSRStorage(size=9, reset=0,
                                   description="Clock output 1 phase offset, in degrees")
        self.Auto_PHcfg_smpls = CSRStorage(size=16, reset=0xEFFF,
                                           description="Number of samples to use during auto phase configuration")


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