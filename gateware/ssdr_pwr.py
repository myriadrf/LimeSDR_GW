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

# SSDR PWR ----------------------------------------------------------------------------------------

class ssdr_pwr(LiteXModule):
    def __init__(self, platform, pads):
        # Add CSRs
        self.ldoen = CSRStorage(1, reset=0,
            description="LDO enable"
        )

        self.comb += pads.ldoen.eq(self.ldoen.storage)



