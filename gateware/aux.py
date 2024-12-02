#
# This file is part of LiteX-XTRX.
#
# Copyright (c) 2021-2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

from litex.soc.cores.spi import SPIMaster

# AUX ----------------------------------------------------------------------------------------------

class AUX(LiteXModule):
    def __init__(self, pads):
        # CSRs.
        self.control = CSRStorage(fields=[
            CSRField("iovcc_sel",  size=1, offset=0, reset=0),
            CSRField("en_smsigio", size=1, offset=1, reset=0),
            CSRField("option", size=1, offset=2, reset=0),
            CSRField("gpio13", size=1, offset=3, reset=0),
        ])

        # # #

        # Drive Control Pins.

        if hasattr(pads, "iovcc_sel"):
            self.comb += pads.iovcc_sel.eq(self.control.fields.iovcc_sel) # FIXME: Check polarity.
        if hasattr(pads, "option"):
            self.comb += pads.option.eq(self.control.fields.option), # FIXME: Check polarity.

        self.comb += [
            pads.en_smsigio.eq(self.control.fields.en_smsigio), # FIXME: Check polarity.
            pads.gpio13.eq(self.control.fields.gpio13), # FIXME: Check polarity.
        ]
