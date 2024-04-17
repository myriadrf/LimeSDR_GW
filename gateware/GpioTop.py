#!/usr/bin/env python3

from migen import *
from litex.soc.interconnect.csr import *


class GpioTop(Module, AutoCSR):
    def __init__(self, platform, pads):
        # CSRs
        self.control = CSRStorage(fields=[
            CSRField("gpio_override", size = len(pads),  offset=0*len(pads), values=[
            ("``0b0``", "GPIO normal operation"),
            ("``0b1``", "GPIO control is overriden")
            ], reset=0),
            CSRField("gpio_override_dir", size=len(pads), offset=1*len(pads), values=[
                ("``0b0``", "Output"),
                ("``0b1``", "Input")
            ], reset=0),
            CSRField("gpio_override_val", size=len(pads), offset=2*len(pads), values=[
                ("``0b0``", "Set output to logic High, dir must be set to output"),
                ("``0b1``", "Set output to logic Low, dir must be set to output")
            ], reset=0)
        ])

        self.status   = CSRStatus(fields=[
            CSRField("gpio_val", size=len(pads), offset=0*len(pads), values=[
                ("GPIO current value")
            ], reset=0)
        ])

        self.GPIO_DIR       = Signal(len(pads))
        self.GPIO_OUT_VAL   = Signal(len(pads))
        self.GPIO_IN_VAL    = Signal(len(pads))
        self.GPIO_I = Signal(len(pads))
        self.GPIO_O = Signal(len(pads))
        self.GPIO_T = Signal(len(pads))

        ###

        platform.add_source("./gateware/LimeIP_HDL/gpio_top/src/gpio_top.vhd")

        # Create params
        self.params_ios = dict()

        # Assign generics
        self.params_ios.update(
            p_G_GPIO_WIDTH=len(pads)
        )

        # Assign ports
        self.params_ios.update(
            i_GPIO_DIR          =self.GPIO_DIR,
            i_GPIO_OUT_VAL      =self.GPIO_OUT_VAL,
            o_GPIO_IN_VAL       =self.GPIO_IN_VAL,
            i_GPIO_OVERRIDE     =self.control.fields.gpio_override,
            i_GPIO_OVERRIDE_DIR =self.control.fields.gpio_override_dir,
            i_GPIO_OVERRIDE_VAL =self.control.fields.gpio_override_val,
            i_GPIO_I            =self.GPIO_I,
            o_GPIO_O            =self.GPIO_O,
            o_GPIO_T            =self.GPIO_T
        )
        # Assign GPIO current value to status register
        self.comb += self.status.fields.gpio_val.eq(self.GPIO_IN_VAL)

        # Create instance and assign params
        self.specials += Instance("gpio_top", **self.params_ios)

        # Xilinx bidirectional buffer primitive
        for n in range(len(pads)):
            self.specials += Instance("IOBUF",
                o_O     = self.GPIO_I[n],
                io_IO   = pads[n],
                i_I     = self.GPIO_O[n],
                i_T     = self.GPIO_T[n]
            )
