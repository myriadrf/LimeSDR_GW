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
    # Clk.
    ("LMK_CLK", 0, Pins("B12"), IOStandard("2.5 V")),

    # Leds.
    ("FPGA_LED1_G", 0, Pins("D2"),  IOStandard("3.3-V LVCMOS")),
    ("FPGA_LED1_R", 0, Pins("E3"),  IOStandard("3.3-V LVCMOS")),
    ("FPGA_LED2_G", 0, Pins("J4"),  IOStandard("3.3-V LVCMOS")),
    ("FPGA_LED2_R", 0, Pins("E1"),  IOStandard("3.3-V LVCMOS")),
    ("FX3_LED_G", 0, Pins("G5"),  IOStandard("3.3-V LVCMOS")),
    ("FX3_LED_R", 0, Pins("H5"),  IOStandard("3.3-V LVCMOS")),

    # Revision.
    ("revision", 0,
        Subsignal("HW_VER",  Pins("F20 F19 G18 H17"), IOStandard("2.5 V"), Misc("WEAK_PULL_UP_RESISTOR ON")),
        Subsignal("BOM_VER", Pins("P1 R2 U2 U1"),     IOStandard("1.8 V")),
    ),

    # GPIO.
    ("FPGA_GPIO", 0, Pins("H8 H6 H2 H1 G4 G3 F2 F1"), IOStandard("3.3-V LVCMOS")),

    # I2C.
    ("FPGA_I2C", 0,
        Subsignal("scl", Pins("H7")),
        Subsignal("sda", Pins("J7")),
        IOStandard("3.3-V LVCMOS"),
        Misc("WEAK_PULL_UP_RESISTOR ON")
    ),

    # SPI.
    ("FPGA_SPI0", 0, # LMS
        Subsignal("clk",  Pins("E6")),
        Subsignal("cs_n", Pins("D10")),
        Subsignal("mosi", Pins("D7")),
        Subsignal("miso", Pins("C8")),
        IOStandard("2.5 V")
    ),
    ("FPGA_SPI1", 0, # ADF/DAC
        Subsignal("clk",  Pins("K8")),
        Subsignal("cs_n", Pins("J5")),
        Subsignal("cs_n", Pins("J3"), Misc("WEAK_PULL_UP_RESISTOR ON")),
        Subsignal("mosi", Pins("L8")),
        IOStandard("3.3-V LVCMOS")
    ),

    # ADF MUXOUT.
    ("ADF_MUXOUT", 0, Pins("J2"), IOStandard("3.3-V LVCMOS")),

    # USB 3.0 (FX3).
    # PCLK declared separately to be able to request it from the platform
    # without requesting the other FX3 signals
    ("FX3_PCLK", 0, Pins("T21"), IOStandard("1.8V")),
    ("FX3", 0,
        Subsignal("dq",   Pins(
            "M19 AA21 Y22 Y21 W22 W21 W20 V22 V21 U22 U21 U20 U19 M22 M21 R22",
            "R21 R20 R19 R18 P22 P21 M20 P16 P15 N22 N21 N20 N19 N18 N17 N16"
        ), Misc("FAST_OUTPUT_REGISTER ON"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl0", Pins("L6")),
        Subsignal("ctl1", Pins("L7"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl2", Pins("M1")),
        Subsignal("ctl3", Pins("M2"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl4", Pins("M3")),
        Subsignal("ctl5", Pins("M4")),
        Subsignal("ctl7", Pins("M7"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl8", Pins("M8")),
        Subsignal("ctl11", Pins("N5"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        Subsignal("ctl12", Pins("N6"), Misc("CLOCK_TO_OUTPUT_DELAY 1")),
        IOStandard("1.8 V"),
    ),

    # RF-IC / LMS7002M.
    ("LMS", 0,
        # Control.
        Subsignal("RESET",       Pins("C6")),
        Subsignal("RXEN",        Pins("C3")),
        Subsignal("TXEN",        Pins("B10")),
        Subsignal("CORE_LDO_EN", Pins("B18")),

        # RX Interface (LMS -> FPGA).
        Subsignal("DIQ1_D",      Pins("B17 B16 B15 B14 B13 C13 A18 A17 A16 A15 A14 A13"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("TXNRX1",      Pins("B9")),
        Subsignal("IQSEL1",      Pins("C4"), Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("MCLK1",       Pins("G21")),
        Subsignal("FCLK1",       Pins("B20"), Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),

        # TX Interface (FPGA -> LMS).
        Subsignal("DIQ2_D",      Pins("B7 B6 B4 B3 A10 A9 A8 A7 A6 A5 A4 A3"),
            Misc("CURRENT_STRENGTH_NEW \"MINIMUM CURRENT\"")),
        Subsignal("TXNRX2",      Pins("B8")),
        Subsignal("IQSEL2",      Pins("C7"),  Misc("CURRENT_STRENGTH_NEW \"MINIMUM CURRENT\"")),
        Subsignal("MCLK2",       Pins("B11"), Misc("CURRENT_STRENGTH_NEW \"MINIMUM CURRENT\"")),
        Subsignal("FCLK2",       Pins("E5")),

        # IOStandard.
        IOStandard("2.5 V")
    ),

    # DDRAM.
    ("ddram", 0,
        Subsignal("a", Pins(
            "W1 AA3 Y3 V6 AA1 AB3 Y2 AB5 Y1 V7 Y6 W2 T8"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("ba",    Pins("V4 V3 V1"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("cas_n", Pins("T4"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("cke",   Pins("R7"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("clk",   Pins("U7"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("PAD_TO_CORE_DELAY 0")),
        Subsignal("clk_n", Pins("U8"),
            Misc("CURRENT_STRENGTH_NEW 12MA")),
        Subsignal("cs_n",  Pins("R6"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("dm",    Pins("AA7 V5"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("dq", Pins(
            "U10 AB8 AB7 AA9 V11 W10 AA8 Y10",
            "W6 AA5 W7 V8 W8 Y7 U9 AA4"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("dqs",   Pins("AB9 V10"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("odt",   Pins("P6"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("ras_n", Pins("T5"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("we_n",  Pins("T7"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        IOStandard("SSTL-18 Class I")
    ),
    ("ddram", 1,
        Subsignal("a", Pins(
            "U15 AA17 T14 Y17 T16 P7 U17 N8 R16 AA20 AB17 U16 U14"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("ba",    Pins("U13 T13 V2"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("cas_n", Pins("T10"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("cke",   Pins("AB10"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("clk",   Pins("R14"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("PAD_TO_CORE_DELAY 0")),
        Subsignal("clk_n", Pins("R15"),
            Misc("CURRENT_STRENGTH_NEW 12MA")),
        Subsignal("cs_n",  Pins("Y8"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("dm",    Pins("AA16 AA10"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("dq", Pins(
            "W15 V14 AB20 AB18 T15 W17 AB16 V15",
            "W13 AB13 AA15 AB14 AA14 AB15 AA13 U12"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("dqs",   Pins("V13 Y13"),
            Misc("CURRENT_STRENGTH_NEW 12MA"),
            Misc("MEM_INTERFACE_DELAY_CHAIN_CONFIG FLEXIBLE_TIMING"),
            Misc("OUTPUT_ENABLE_GROUP 3078784")),
        Subsignal("odt",   Pins("T12"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("ras_n", Pins("T11"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        Subsignal("we_n",  Pins("T9"),
            Misc("CURRENT_STRENGTH_NEW \"MAXIMUM CURRENT\"")),
        IOStandard("SSTL-18 Class I")
    ),

    # RF Loopback Control.
    ("LB", 0,
        Subsignal("TX1_AT", Pins("E16")),
        Subsignal("TX1_H",  Pins("F15")),
        Subsignal("TX1_L",  Pins("F14")),
        Subsignal("TX1_SH", Pins("G15")),
        Subsignal("TX2_AT", Pins("E15")),
        Subsignal("TX2_H",  Pins("H11")),
        Subsignal("TX2_L",  Pins("F11")),
        Subsignal("TX2_SH", Pins("F16")),
        IOStandard("2.5 V")
    ),

    # Temperature Sensor.
    ("LM75_OS", 0, Pins("J6"), IOStandard("3.3-V LVCMOS")),

    # Fan Control.
    ("FAN_CTRL", 0, Pins("E4"), IOStandard("3.3-V LVCMOS")),

    # Bridge SPI.
    ("BRDG_SPI", 0,
        Subsignal("cs_n", Pins("K7")),
        Subsignal("miso", Pins("C1")),
        Subsignal("mosi", Pins("B1")),
        Subsignal("clk",  Pins("B2")),
        IOStandard("3.3-V LVCMOS")
    ),

    # Clock Generator (Si5351).
    ("SI_CLK", 0, Pins("T2"),    IOStandard("1.8 V")),
    ("SI_CLK", 1, Pins("AA12"),  IOStandard("1.8 V")),
    ("SI_CLK", 2, Pins("AB12"),  IOStandard("1.8 V")),
    ("SI_CLK", 3, Pins("T22"),   IOStandard("1.8 V")),
    ("SI_CLK", 5, Pins("G1"),    IOStandard("3.3-V LVCMOS")),
    ("SI_CLK", 6, Pins("AA11"),  IOStandard("1.8 V")),
    ("SI_CLK", 7, Pins("AB11"),  IOStandard("1.8 V")),

    ("EXT_GND", 0, Pins("T17"), IOStandard("1.8 V")),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(AlteraPlatform):
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
        # LMS_DIQ1_D Timing Delays (from QSF)
        try:
            self.lookup_request("LMS")
            diq1_delays = [1, 0, 1, 1, 1, None, 1, 0, 0, 1, 1, 0]
            for i, delay in enumerate(diq1_delays):
                if delay is not None:
                    self.add_platform_command(f"set_instance_assignment -name CLOCK_TO_OUTPUT_DELAY {delay} -to LMS_DIQ1_D[{i}]")
        except:
            pass

        # CKN_CK_PAIR for DDRAM (from QSF)
        try:
            self.lookup_request("ddram", 0)
            self.add_platform_command("set_instance_assignment -name CKN_CK_PAIR ON -from ddram_clk_n -to ddram_clk")
        except:
            pass

        try:
            self.lookup_request("ddram", 1)
            self.add_platform_command("set_instance_assignment -name CKN_CK_PAIR ON -from ddram_1_clk_n -to ddram_1_clk")
        except:
            pass
