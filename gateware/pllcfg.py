#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# PLL Cfg ------------------------------------------------------------------------------------------

class PLLCfg(LiteXModule):
    def __init__(self):
        self.auto_phcfg_smpls  = Signal(16)
        self.pll_lock          = CSRStatus(16,  reset=0)
        self._auto_phcfg_smpls = CSRStorage(16, reset=0x3FF)
        self.auto_phcfg_step   = CSRStorage(16, reset=0x002)

        # # #

        # Logic.
        self.comb += [
            self.pll_lock.status.eq(    Constant(0, 16)),
            self.auto_phcfg_smpls.eq(self._auto_phcfg_smpls.storage),
        ]
