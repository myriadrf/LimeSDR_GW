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
        self.PHCFG_MODE = CSRStorage(size=1, reset=0,
                                     description="Phase configuration mode: 0: Manual, 1: Auto")
        self.PHCFG_DONE = CSRStorage(size=1, reset=0,
                                     description="Phase config done: 0: Not done, 1: Done  ")
        self.PHCFG_ERR  = CSRStorage(size=1, reset=0)
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
        self.mmcm.create_clkout(self.cd_clkout0, max_freq)
        self.mmcm.create_clkout(logic_cd, max_freq)
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
