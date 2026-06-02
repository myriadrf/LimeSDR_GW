#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from litex.build.generic_platform import *
from litex.build.altera import AlteraPlatform
from litex.build.openfpgaloader import OpenFPGALoader

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # TODO: Define IOs for LimeSDR USB based on pinout.

    # Clk.
    # ("clk50", 0, Pins("???"), IOStandard("3.3-V LVCMOS")),

    # Leds.
    # ("user_led", 0, Pins("???"), IOStandard("3.3-V LVCMOS")),

    # USB-FIFO .
    # ("fx2", 0, ...),

    # RF-IC / LMS7002M.
    # ("lms", 0, ...),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(AlteraPlatform):
    default_clk_name   = "clk50"
    default_clk_period = 1e9/50e6
    create_rbf         = False

    def __init__(self, device="EP4CE40F23C8", **kwargs):
        AlteraPlatform.__init__(self, device, _io, **kwargs)

        # FPGA device/bitstream parameters.
        self.add_platform_command("set_global_assignment -name DEVICE_FILTER_PACKAGE FBGA")
        self.add_platform_command("set_global_assignment -name DEVICE_FILTER_PIN_COUNT 484")
        self.add_platform_command("set_global_assignment -name DEVICE_FILTER_SPEED_GRADE 8")

    def create_programmer(self, cable="ft2232"):
        return OpenFPGALoader(cable=cable)

    def do_finalize(self, fragment):
        # self.add_period_constraint(...)
        pass
