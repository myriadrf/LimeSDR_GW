#
# This file is part of LimeSDR_GW.
#
# Copyright (c) 2024-2025 Lime Microsystems.
#
# SPDX-License-Identifier: Apache-2.0

from litex.build.generic_platform import *
from litex.build.lattice import LatticeECP5Platform
from litex.build.openfpgaloader import OpenFPGALoader

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk.
    ("LMK_CLK", 0, Pins("A9"), IOStandard("LVCMOS33")),

    # Leds.
    ("FPGA_LED1_G", 0, Pins("R16"), IOStandard("LVCMOS33"), Misc("OPENDRAIN=ON")),
    ("FPGA_LED1_R", 0, Pins("V17"), IOStandard("LVCMOS33"), Misc("OPENDRAIN=ON")),

    # Revision.
    ("revision", 0,
        Subsignal("HW_VER",  Pins("D4 M2 N4 J3"), Misc("PULLMODE=UP")),
        Subsignal("BOM_VER", Pins("N1 M1 N2")),
        IOStandard("LVCMOS25"),
    ),

    # # Serial/Debug.
    # ("serial", 0,
    #     Subsignal("tx", Pins("A10")), # FPGA_EGPIO0.
    #     Subsignal("rx", Pins("A8")),  # FPGA_EGPIO1.
    #     IOStandard("LVCMOS33")
    # ),

    # GPIO.
    ("FPGA_GPIO",  0, Pins("N15 N18 N16 N17 M18 R18 T17 R17"), IOStandard("LVCMOS33")),
    ("FPGA_EGPIO", 0, Pins("A10 A8"),                          IOStandard("LVCMOS33")),

    # SPIFlash
    ("spiflash", 0,
        Subsignal("cs_n", Pins("U17")),
        #Subsignal("clk",  Pins("U16")),
        Subsignal("miso", Pins("T18")),
        Subsignal("mosi", Pins("U18")),
        IOStandard("LVCMOS33"),
    ),

    # I2C.
    ("FPGA_I2C", 0,
        Subsignal("scl", Pins("C10"), Misc("OPENDRAIN=ON")),
        Subsignal("sda", Pins("B9"),  Misc("OPENDRAIN=ON")),
        IOStandard("LVCMOS33"),
    ),

    # SPI.
    ("FPGA_SPI", 0,
        Subsignal("clk",    Pins("M3")),
        #Subsignal("LMS_SS", Pins("N3")),
        #Subsignal("DAC_SS", Pins("L4")),
        Subsignal("cs_n",   Pins("N3 L4")),
        Subsignal("mosi",   Pins("L3")),
        Subsignal("miso",   Pins("K3")),
        IOStandard("LVCMOS25"),
    ),

    # Temperature Sensor.
    ("LM75_OS", 0, Pins("K2"), IOStandard("LVCMOS25")),

    # Fan Control.
    ("FAN_CTRL", 0, Pins("A11"), IOStandard("LVCMOS33")),

    # USB-FIFO.
    ("FT_CLK", 0, Pins("D17"), IOStandard("LVCMOS33")),
    ("FT", 0,
        Subsignal("RESETn",  Pins("M17")),
        Subsignal("D",       Pins(
            "A13 B12 B15 C12 A16 A12 D18 B17",
            "F15 D16 D15 C13 H18 B13 J18 A15",
            "B18 C18 A17 K18 C15 L18 F18 C16",
            "G16 D13 G18 F16 C17 F17 K15 K17")),
        Subsignal("BE",      Pins("L15 J17 K16 H17")),
        Subsignal("RXFn",    Pins("H16")),
        Subsignal("TXEn",    Pins("M16")),
        Subsignal("WRn",     Pins("J16")),
        Subsignal("WAKEUPn", Pins("G15")),
        #Subsignal("rd_n",    Pins("H15")),
        #Subsignal("oe_n",    Pins("L16")),
        IOStandard("LVCMOS33"),
    ),

    # RF-IC / LMS7002M.
    ("LMS", 0,
        # Control.
        Subsignal("RESET",            Pins("A7")),
        Subsignal("CORE_LDO_EN",      Pins("C6")),
        Subsignal("RXEN",             Pins("D6")),
        Subsignal("TXEN",             Pins("B7")),

        # RX Interface (LMS -> FPGA).
        Subsignal("DIQ1_D",            Pins("J2 L1 K1 K4 G3 F4 J1 H1 G4 F2 G1 H2")),
        Subsignal("TXNRX1",            Pins("F1")),
        Subsignal("ENABLE_IQSEL1",     Pins("F3")),
        Subsignal("MCLK1",             Pins("H4")),
        Subsignal("FCLK1",             Pins("H3")),

        # TX Interface (FPGA -> LMS).
        Subsignal("DIQ2_D",            Pins("A3 C2 A2 B4 C3 B2 D3 B1 A4 C1 C7 A6")),
        Subsignal("TXNRX2_or_CLK_SEL", Pins("B6")),
        Subsignal("ENABLE_IQSEL2",     Pins("C4")),
        Subsignal("MCLK2",             Pins("D2")),
        Subsignal("FCLK2",             Pins("D1")),

        # IOStandard/Slew Rate.
        IOStandard("LVCMOS25"),
    ),

    # RF Loopback Control.
    ("RFSW", 0,
        Subsignal("RX_V1", Pins("C11")),
        Subsignal("RX_V2", Pins("B11")),
        Subsignal("TX_V1", Pins("B10")),
        Subsignal("TX_V2", Pins("C9")),
        IOStandard("LVCMOS33"),
    ),
    ("TX_LB", 0,
        Subsignal("AT", Pins("C8")),
        Subsignal("SH", Pins("B8")),
        IOStandard("LVCMOS33")
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(LatticeECP5Platform):
    default_clk_name   = "LMK_CLK"
    default_clk_period = 1e9/40e6

    def __init__(self, device="LFE5U", toolchain="diamond", **kwargs):
        assert device in ["LFE5U"]
        LatticeECP5Platform.__init__(self, device + "-45F-8MG285C", _io, toolchain=toolchain, **kwargs)

        # FPGA device/bitstream parameters.
        # FIXME.

    def create_programmer(self, cable="ft2232"):
        return OpenFPGALoader(cable=cable)

    def do_finalize(self, fragment):
        self.add_period_constraint(self.lookup_request("FT_CLK",    loose=True), 1e9/100e6)
        self.add_period_constraint(self.lookup_request("LMK_CLK",   loose=True), 1e9/40e6)
        self.add_period_constraint(self.lookup_request("LMS:MCLK1", loose=True), 1e9/125e6)
        self.add_period_constraint(self.lookup_request("LMS:MCLK2", loose=True), 1e9/125e6)
