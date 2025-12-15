#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from migen import *
from litex.gen import *
from litex.soc.interconnect.csr import *

# GPIO Top -----------------------------------------------------------------------------------------

class GpioTop(LiteXModule):
    def __init__(self, platform, pads):
        # Make sure pads is a vector-like object from platform
        npads = len(pads)

        self.GPIO_DIR     = Signal(npads)
        self.GPIO_OUT_VAL = Signal(npads)
        self.GPIO_IN_VAL  = Signal(npads)

        # CSRs
        self.gpio_override = CSRStorage(npads,
            description="GPIO Mode: 0: normal operation, 1: control is overriden."
        )
        self.gpio_override_dir = CSRStorage(npads, reset=(2**npads - 1),
            description="GPIO override direction: 0: Output, 1: Input."
        )
        self.gpio_override_val = CSRStorage(npads,
            description="GPIO Logic level: 0: High, 1: Low. (Dir must be set to output)"
        )
        self.gpio_val = CSRStatus(size=npads, description="GPIO current value")

        # # #

        # Create one TSTriple vector (works correctly in Verilog)
        # Correct: one TSTriple per pad
        self.triples = []
        for n, pad in enumerate(pads):
            t = TSTriple()
            self.triples.append(t)
            self.specials += t.get_tristate(pad)

        # Combine into bus signals
        self.GPIO_I = Cat(*[t.i for t in self.triples])
        self.GPIO_O = Cat(*[t.o for t in self.triples])
        self.GPIO_OE = Cat(*[t.oe for t in self.triples])

        # Assign current input value to CSR
        self.comb += [
            self.gpio_val.status.eq(self.GPIO_IN_VAL)
        ]

        # Instantiate VHDL GPIO logic
        self.specials += Instance("gpio_top",
            # Parameters
            p_G_GPIO_WIDTH      = npads,

            # Ports
            i_GPIO_DIR          = self.GPIO_DIR,
            i_GPIO_OUT_VAL      = self.GPIO_OUT_VAL,
            o_GPIO_IN_VAL       = self.GPIO_IN_VAL,
            i_GPIO_OVERRIDE     = self.gpio_override.storage,
            i_GPIO_OVERRIDE_DIR = self.gpio_override_dir.storage,
            i_GPIO_OVERRIDE_VAL = self.gpio_override_val.storage,

            # Tri-state interface
            i_GPIO_I            = self.GPIO_I,
            o_GPIO_O            = self.GPIO_O,
            o_GPIO_OE           = self.GPIO_OE
            o_GPIO_OE           = self.GPIO_T
        )

        # Add HDL source
        platform.add_source("./gateware/LimeDFB/gpio_top/src/gpio_top.vhd")
