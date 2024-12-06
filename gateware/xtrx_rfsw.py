#!/usr/bin/env python3

from migen import *
from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *


from litescope import LiteScopeAnalyzer


class xtrx_rfsw(LiteXModule):
    def __init__(self, platform, pads):
        # Add CSRs
        self.tdd_manual_val = CSRStorage(1, reset=0,
            description="TDD Signal manual control value"
        )
        self.tdd_auto_en = CSRStorage(1, reset=0,
            description="0- TDD auto control disabled, 1-TDD auto control enabled"
        )
        self.tdd_invert = CSRStorage(1, reset=0,
            description="0- TDD Control signal not inverted, 1- TDD Control signal inverted"
        )

        self.rfsw_rx = CSRStorage(2, reset=0,
            description="00- RF1 (WIDE), 01- RF2 (LOW), 10- RF3 (HIGH)"
        )

        self.rfsw_tx = CSRStorage(1, reset=0,
            description="0- TX1_2 (BAND2), 1- TX1_1 (BAND1)"
        )

        self.rfsw_auto_en = CSRStorage(1, reset=0,
            description="0- RFSW Auto control disabled, 1- RFSW Auto control Enabled"
        )

        # Add sources
        platform.add_source("./gateware/tdd_control.vhd")

        # create misc signals
        self.AUTO_IN        = Signal()
        self.TDD_OUT        = Signal()

        # Create params
        self.params_ios = dict()

        # Assign ports
        self.params_ios.update(
            # DIQ1
            i_MANUAL_VALUE          = self.tdd_manual_val.storage,
            i_AUTO_ENABLE           = self.tdd_auto_en.storage,
            i_AUTO_IN               = self.AUTO_IN,
            i_AUTO_INVERT           = self.tdd_invert.storage,

            i_RX_RF_SW_IN           = self.rfsw_rx.storage,
            i_TX_RF_SW_IN           = self.rfsw_tx.storage,
            i_RF_SW_AUTO_ENANBLE    = self.rfsw_auto_en.storage,
            o_TDD_OUT               = self.TDD_OUT,
            o_RX_RF_SW_OUT          = pads.rx,
            o_TX_RF_SW_OUT          = pads.tx
        )

        # Create instance and assign params
        self.specials += Instance("tdd_control", **self.params_ios)

