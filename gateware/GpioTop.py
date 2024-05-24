#!/usr/bin/env python3

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

class GpioTop(LiteXModule):
    def __init__(self, platform, pads):
        self.GPIO_DIR     = Signal(len(pads))
        self.GPIO_OUT_VAL = Signal(len(pads))
        self.GPIO_IN_VAL  = Signal(len(pads))

        # CSRs
        self.gpio_override = CSRStorage(len(pads),
            description="GPIO Mode: 0: normal operation, 1: control is overriden."
        )
        self.gpio_override_dir = CSRStorage(len(pads),
            description="GPIO override direction: 0: Output, 1: Input."
        )
        self.gpio_override_val = CSRStorage(len(pads),
            description="GPIO Logic level: 0: High, 1: Low. (Dir must be set to output)"
        )
        self.gpio_val = CSRStatus(size=len(pads), description="GPIO current value")

        # # #

        # Signals.
        self.GPIO_I = Signal(len(pads))
        self.GPIO_O = Signal(len(pads))
        self.GPIO_T = Signal(len(pads))

        # Assign GPIO current value to status register
        self.comb += self.gpio_val.status.eq(self.GPIO_IN_VAL)

        # Create instance and assign params
        self.specials += Instance("gpio_top",
            # Assign generics
            p_G_GPIO_WIDTH      = len(pads),
            # Assign ports
            i_GPIO_DIR          = self.GPIO_DIR,
            i_GPIO_OUT_VAL      = self.GPIO_OUT_VAL,
            o_GPIO_IN_VAL       = self.GPIO_IN_VAL,
            i_GPIO_OVERRIDE     = self.gpio_override.storage,
            i_GPIO_OVERRIDE_DIR = self.gpio_override_dir.storage,
            i_GPIO_OVERRIDE_VAL = self.gpio_override_val.storage,
            i_GPIO_I            = self.GPIO_I,
            o_GPIO_O            = self.GPIO_O,
            o_GPIO_T            = self.GPIO_T
        )

        # Xilinx bidirectional buffer primitive
        for n in range(len(pads)):
            self.specials += Instance("IOBUF",
                o_O     = self.GPIO_I[n],
                io_IO   = pads[n],
                i_I     = self.GPIO_O[n],
                i_T     = self.GPIO_T[n]
            )

        platform.add_source("./gateware/LimeIP_HDL/gpio_top/src/gpio_top.vhd")

