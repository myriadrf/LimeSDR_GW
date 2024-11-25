#
# This file is part of LimeSDR-Mini-v2_GW.
#
# Copyright (c) 2024 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from litex.build.generic_platform import *
from litex.build.altera import AlteraPlatform
from litex.build.altera.programmer import USBBlaster

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk.
    ("LMK_CLK", 0, Pins("H6"), IOStandard("2.5 V")),

    # Leds.
    ("FPGA_LED_G", 0, Pins("D8"), IOStandard("3.3-V LVCMOS")),
    ("FPGA_LED_R", 0, Pins("B9"), IOStandard("3.3-V LVCMOS")),

    # Revision.
    ("revision", 0,
        Subsignal("HW_VER",  Pins("E3 E4 F4 G4")),
        Subsignal("BOM_VER", Pins("K7 H3 C2")),
        Misc("WEAK_PULL_UP_RESISTOR ON"),
        IOStandard("1.8V"),
    ),

    # GPIO.
    ("FPGA_GPIO",  0, Pins("A11 B10 C10 D11 E13 F13 F10 G10"), IOStandard("3.3-V LVCMOS")),
    ("FPGA_EGPIO", 0, Pins("B7 D6"),                           IOStandard("3.3-V LVCMOS")),

    # SPIFlash
    ("spiflash", 0,
        Subsignal("cs_n", Pins("E1")),
        Subsignal("clk",  Pins("F1")),
        Subsignal("mosi", Pins("D1")),
        Subsignal("miso", Pins("C1")),
        IOStandard("1.8V")
    ),

    # I2C.
    ("FPGA_I2C", 0,
        Subsignal("scl", Pins("D13")),
        Subsignal("sda", Pins("K13")),
        IOStandard("3.3-V LVCMOS")
    ),

    # SPI.
    ("FPGA_SPI", 0,
        Subsignal("clk",  Pins("K5")),
        #Subsignal("LMS_SS", Pins("K8")),
        #Subsignal("DAC_SS", Pins("J5")),
        Subsignal("cs_n", Pins("K8 J5")),
        Subsignal("mosi", Pins("J7")),
        Subsignal("miso", Pins("J6")),
        IOStandard("2.5 V")
    ),

    # Temperature Sensor.
    ("LM75_OS", 0, Pins("H2"), IOStandard("1.8V")),

    # Fan Control.
    ("FAN_CTRL", 0, Pins("C5"), IOStandard("3.3-V LVCMOS")),

    # USB-FIFO.
    ("FT_CLK", 0, Pins("G9"), IOStandard("3.3-V LVCMOS")),
    ("FT", 0,
        Subsignal("D", Pins(
            " A2  B6  B3  B5  A3  B4  E6  A4",
            "B12  H8  E9  B2  D9  J9  A9  H9",
            " A7  F8  A6 E10  A5  F9  E8  A8",
            "K11 K12 J12 G12 L13 G13 J13 H13"
        )),
        Subsignal("BE", Pins("B11 C12 A12 B13")),
        Subsignal("RXFn", Pins("C11")),
        Subsignal("TXEn", Pins("C13")),
        Subsignal("WRn", Pins("E12")),
        IOStandard("3.3-V LVCMOS")
    ),

    # RF-IC / LMS7002M.
    ("LMS", 0,
        # Control.
        Subsignal("RESET",        Pins("L1")),
        Subsignal("CORE_LDO_EN",  Pins("N11")),
        Subsignal("RXEN",         Pins("M11")),
        Subsignal("TXEN",         Pins("K6")),

        # RX Interface (LMS -> FPGA).
        Subsignal("DIQ1_D",        Pins("M12 N12 N10 L10 M10 M13 N9 N8 M7 N7 M9 N6")),
        Subsignal("TXNRX1",        Pins("J8")),
        Subsignal("ENABLE_IQSEL1", Pins("M8")),
        Subsignal("MCLK1",         Pins("G5")),
        Subsignal("FCLK1",         Pins("L3")),

        # TX Interface (FPGA -> LMS).
        Subsignal("DIQ2_D",        Pins("M2 M4 M1 J1 N2 K1 L2 J2 N4 K2 L5 L4")),
        Subsignal("TXNRX2",        Pins("H5")),
        Subsignal("ENABLE_IQSEL2", Pins("N3")),
        Subsignal("MCLK2",         Pins("H4")),
        Subsignal("FCLK2",         Pins("M3")),

        # IOStandard/Slew Rate.
        IOStandard("2.5 V")
    ),

    # RF Loopback Control.
    ("RFSW", 0,
        Subsignal("RX_V1", Pins("A10")),
        Subsignal("RX_V2", Pins("C9")),
        Subsignal("TX_V1", Pins("L11")),
        Subsignal("TX_V2", Pins("C4")),
        IOStandard("3.3-V LVCMOS")
    ),
    ("TX_LB", 0,
        Subsignal("AT", Pins("K10")),
        Subsignal("SH", Pins("J10")),
        IOStandard("3.3-V LVCMOS")
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(AlteraPlatform):
    default_clk_name   = "LMK_CLK"
    default_clk_period = 1e9/40e6
    create_rbf         = False

    def __init__(self, device="10M16SAU169C8G", **kwargs):
        AlteraPlatform.__init__(self, device, _io, **kwargs)

    def create_programmer(self):
        return USBBlaster()

    def do_finalize(self, fragment):
        self.add_period_constraint(self.lookup_request("LMK_CLK", loose=True), 1e9/40e6)
